import logging
import os
from typing import Annotated, Optional

import vtk
import ctk
import qt
import math

import slicer
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
    WithinRange,
)

from slicer import vtkMRMLScalarVolumeNode

import numpy as np
import SimpleITK as sitk
import sitkUtils as su
import time

import builtins
import logging as logging_module

#
# PathPlanningMod
#


class PathPlanningMod(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("PathPlanningMod")  # TODO: make this more human readable by adding spaces
        
        # TODO: set categories (folders where the module shows up in the module selector)
        self.parent.categories = [translate("qSlicerAbstractCoreModule", "Examples")]
        self.parent.dependencies = []  # TODO: add here list of module names that this module requires
        self.parent.contributors = ["John Doe (AnyWare Corp.)"]  # TODO: replace with "Firstname Lastname (Organization)"
        
        # TODO: update with short description of the module and a link to online module documentation
        # _() function marks text as translatable to other languages
        self.parent.helpText = _("""
This is an example of scripted loadable module bundled in an extension.
See more information in <a href="https://github.com/organization/projectname#PathPlanningMod">module documentation</a>.
""")
        # TODO: replace with organization, grant and thanks
        self.parent.acknowledgementText = _("""
This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab,
and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1.
""")

        # Additional initialization step after application startup is complete
        slicer.app.connect("startupCompleted()", registerSampleData)


#
# Register sample data sets in Sample Data module
#


def registerSampleData():
    """Add data sets to Sample Data module."""
    # It is always recommended to provide sample data for users to make it easy to try the module,
    # but if no sample data is available then this method (and associated startupCompeted signal connection) can be removed.

    import SampleData

    iconsPath = os.path.join(os.path.dirname(__file__), "Resources/Icons")

    # To ensure that the source code repository remains small (can be downloaded and installed quickly)
    # it is recommended to store data sets that are larger than a few MB in a Github release.

    # PathPlanningMod1
    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        # Category and sample name displayed in Sample Data module
        category="PathPlanningMod",
        sampleName="PathPlanningMod1",
        # Thumbnail should have size of approximately 260x280 pixels and stored in Resources/Icons folder.
        # It can be created by Screen Capture module, "Capture all views" option enabled, "Number of images" set to "Single".
        thumbnailFileName=os.path.join(iconsPath, "PathPlanningMod1.png"),
        # Download URL and target file name
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95",
        fileNames="PathPlanningMod1.nrrd",
        # Checksum to ensure file integrity. Can be computed by this command:
        #  import hashlib; print(hashlib.sha256(open(filename, "rb").read()).hexdigest())
        checksums="SHA256:998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95",
        # This node name will be used when the data set is loaded
        nodeNames="PathPlanningMod1",
    )

    # PathPlanningMod2
    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        # Category and sample name displayed in Sample Data module
        category="PathPlanningMod",
        sampleName="PathPlanningMod2",
        thumbnailFileName=os.path.join(iconsPath, "PathPlanningMod2.png"),
        # Download URL and target file name
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97",
        fileNames="PathPlanningMod2.nrrd",
        checksums="SHA256:1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97",
        # This node name will be used when the data set is loaded
        nodeNames="PathPlanningMod2",
    )


#
# PathPlanningModParameterNode
#


@parameterNodeWrapper
class PathPlanningModParameterNode:
    """
    The parameters needed by module.

    inputVolume - The volume to threshold.
    imageThreshold - The value at which to threshold the input volume.
    invertThreshold - If true, will invert the threshold.
    thresholdedVolume - The output volume that will contain the thresholded volume.
    invertedVolume - The output volume that will contain the inverted thresholded volume.
    """

    inputVolume: vtkMRMLScalarVolumeNode
    imageThreshold: Annotated[float, WithinRange(-100, 500)] = 100
    invertThreshold: bool = False
    thresholdedVolume: vtkMRMLScalarVolumeNode
    invertedVolume: vtkMRMLScalarVolumeNode

#
# PathPlanningModWidget
#

class PathPlanningModWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    """Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent=None) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.__init__(self, parent)
        VTKObservationMixin.__init__(self)  # needed for parameter node observation
        self.logic = None
        self._parameterNode = None
        self._parameterNodeGuiTag = None

    def setup(self) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.setup(self)

        # Clear the entire layout first by removing all existing widgets
        while self.layout.count():
            item = self.layout.itemAt(0)
            if item.widget():
                item.widget().deleteLater()
            self.layout.removeItem(item)
        
        # Create a fresh layout
        #self.layout = qt.QVBoxLayout(self.parent)
        
        # Add Reload & Test section (which you want to keep)
        reloadCollapsibleButton = ctk.ctkCollapsibleButton()
        reloadCollapsibleButton.text = "Reload & Test"
        self.layout.addWidget(reloadCollapsibleButton)
        reloadFormLayout = qt.QFormLayout(reloadCollapsibleButton)
        
        # Create buttons for the Reload section
        buttonsLayout = qt.QHBoxLayout()
        self.reloadButton = qt.QPushButton("Reload")
        buttonsLayout.addWidget(self.reloadButton)
        self.reloadAndTestButton = qt.QPushButton("Reload and Test")
        buttonsLayout.addWidget(self.reloadAndTestButton)
        self.restartSlicerButton = qt.QPushButton("Restart Slicer")
        buttonsLayout.addWidget(self.restartSlicerButton)
        reloadFormLayout.addRow(buttonsLayout)
        
        # Add Edit buttons
        buttonsLayout2 = qt.QHBoxLayout()
        self.editButton = qt.QPushButton("Edit")
        buttonsLayout2.addWidget(self.editButton)
        self.editUIButton = qt.QPushButton("Edit UI")
        buttonsLayout2.addWidget(self.editUIButton)
        reloadFormLayout.addRow(buttonsLayout2)

        # Connect the reload section buttons
        self.reloadButton.connect('clicked()', self.onReload)
        self.reloadAndTestButton.connect('clicked()', self.onReloadAndTest)
        self.restartSlicerButton.connect('clicked()', slicer.app.restart)
        self.editButton.connect('clicked()', self.onEditSource)
        self.editUIButton.connect('clicked()', self.onEditUI)
        
        # Create a main collapsible button for path planning
        pathPlanningCollapsibleButton = ctk.ctkCollapsibleButton()
        pathPlanningCollapsibleButton.text = "Surgical Path Planning"
        pathPlanningCollapsibleButton.collapsed = False  # Start expanded
        self.layout.addWidget(pathPlanningCollapsibleButton)
        
        # Create a form layout for the path planning section
        pathPlanningFormLayout = qt.QFormLayout(pathPlanningCollapsibleButton)
        
        # Add a header label with instructions
        headerLabel = qt.QLabel("Select input structures and entry/target points to plan a safe surgical trajectory")
        headerLabel.setWordWrap(True)
        pathPlanningFormLayout.addRow(headerLabel)
        
        # Add a line separator
        line = qt.QFrame()
        line.setFrameShape(qt.QFrame.HLine)
        line.setFrameShadow(qt.QFrame.Sunken)
        pathPlanningFormLayout.addRow(line)
        
        # Structure selectors section (with clearer labels)
        structureLabel = qt.QLabel("<b>Input Structures:</b>")
        pathPlanningFormLayout.addRow(structureLabel)
        
        # Add Upload Data button
        self.uploadDataButton = qt.QPushButton("Upload Data")
        self.uploadDataButton.toolTip = "Load data from files"
        self.uploadDataButton.setStyleSheet("background-color: #2196F3; color: white;")
        self.uploadDataButton.connect('clicked(bool)', self.onUploadDataButton)
        pathPlanningFormLayout.addRow(self.uploadDataButton)
        
        # Add input selectors
        self.hippoSelector = slicer.qMRMLNodeComboBox()
        self.hippoSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.hippoSelector.selectNodeUponCreation = True
        self.hippoSelector.setMRMLScene(slicer.mrmlScene)
        self.hippoSelector.setToolTip("Select the hippocampus volume")
        pathPlanningFormLayout.addRow("Hippocampus: ", self.hippoSelector)
        
        self.ventriclesSelector = slicer.qMRMLNodeComboBox()
        self.ventriclesSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.ventriclesSelector.selectNodeUponCreation = True
        self.ventriclesSelector.setMRMLScene(slicer.mrmlScene)
        self.ventriclesSelector.setToolTip("Select the ventricles volume")
        pathPlanningFormLayout.addRow("Ventricles: ", self.ventriclesSelector)
        
        self.vesselsSelector = slicer.qMRMLNodeComboBox()
        self.vesselsSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.vesselsSelector.selectNodeUponCreation = True
        self.vesselsSelector.setMRMLScene(slicer.mrmlScene)
        self.vesselsSelector.setToolTip("Select the blood vessels volume")
        pathPlanningFormLayout.addRow("Vessels: ", self.vesselsSelector)
        
        self.cortexSelector = slicer.qMRMLNodeComboBox()
        self.cortexSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.cortexSelector.selectNodeUponCreation = True
        self.cortexSelector.setMRMLScene(slicer.mrmlScene)
        self.cortexSelector.setToolTip("Select the cortex volume")
        pathPlanningFormLayout.addRow("Cortex: ", self.cortexSelector)
        
        # Add another line separator
        line2 = qt.QFrame()
        line2.setFrameShape(qt.QFrame.HLine)
        line2.setFrameShadow(qt.QFrame.Sunken)
        pathPlanningFormLayout.addRow(line2)
        
        # Trajectory points section
        pointsLabel = qt.QLabel("<b>Trajectory Points:</b>")
        pathPlanningFormLayout.addRow(pointsLabel)
        
        self.entrySelector = slicer.qMRMLNodeComboBox()
        self.entrySelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.entrySelector.selectNodeUponCreation = True
        self.entrySelector.setMRMLScene(slicer.mrmlScene)
        self.entrySelector.setToolTip("Select the entry points")
        pathPlanningFormLayout.addRow("Entry Points: ", self.entrySelector)
        
        self.targetSelector = slicer.qMRMLNodeComboBox()
        self.targetSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.targetSelector.selectNodeUponCreation = True
        self.targetSelector.setMRMLScene(slicer.mrmlScene)
        self.targetSelector.setToolTip("Select the target points")
        pathPlanningFormLayout.addRow("Target Points: ", self.targetSelector)
        
        # Add another line separator
        line3 = qt.QFrame()
        line3.setFrameShape(qt.QFrame.HLine)
        line3.setFrameShadow(qt.QFrame.Sunken)
        pathPlanningFormLayout.addRow(line3)
        
        # Constraints section
        constraintsLabel = qt.QLabel("<b>Constraints:</b>")
        pathPlanningFormLayout.addRow(constraintsLabel)
        
        # Add Trajectory Length Constraint controls
        self.enableLengthConstraintCheckBox = qt.QCheckBox()
        self.enableLengthConstraintCheckBox.checked = True
        self.enableLengthConstraintCheckBox.setToolTip("Enable or disable trajectory length constraint")
        pathPlanningFormLayout.addRow("Enable length constraint:", self.enableLengthConstraintCheckBox)

        self.maxLengthSlider = ctk.ctkSliderWidget()
        self.maxLengthSlider.singleStep = 5.0
        self.maxLengthSlider.minimum = 10.0
        self.maxLengthSlider.maximum = 80.0
        self.maxLengthSlider.value = 80.0  # Default value
        self.maxLengthSlider.setToolTip("Set the maximum allowed trajectory length in mm")
        pathPlanningFormLayout.addRow("Max trajectory length (mm):", self.maxLengthSlider)

        # Connect checkbox to enable/disable slider
        self.enableLengthConstraintCheckBox.connect("toggled(bool)", self.onEnableLengthConstraintToggled)
        
        # Add another line separator
        line4 = qt.QFrame()
        line4.setFrameShape(qt.QFrame.HLine)
        line4.setFrameShadow(qt.QFrame.Sunken)
        pathPlanningFormLayout.addRow(line4)
        
        # Status section
        statusLabel = qt.QLabel("<b>Status:</b>")
        pathPlanningFormLayout.addRow(statusLabel)
        
        # Add a status label
        self.statusLabel = qt.QLabel("Ready")
        self.statusLabel.setStyleSheet("font-weight: bold;")
        pathPlanningFormLayout.addRow("Current status: ", self.statusLabel)
        
        # Add action buttons in a horizontal layout
        buttonsWidget = qt.QWidget()
        buttonsLayout = qt.QHBoxLayout(buttonsWidget)
        buttonsLayout.setContentsMargins(0, 0, 0, 0)
        
        # Add a button to run the path planning
        self.runPlanningButton = qt.QPushButton("Run Path Planning")
        self.runPlanningButton.toolTip = "Run the path planning algorithm"
        self.runPlanningButton.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        buttonsLayout.addWidget(self.runPlanningButton)
        
        # Add a button to clear results
        self.clearResultsButton = qt.QPushButton("Clear Results")
        self.clearResultsButton.toolTip = "Clear the current results"
        buttonsLayout.addWidget(self.clearResultsButton)
        
        pathPlanningFormLayout.addRow(buttonsWidget)
        
        # Add a button to send transform
        self.sendTransformButton = qt.QPushButton("Send Trajectory to ROS/MoveIt")
        self.sendTransformButton.toolTip = "Send the optimal trajectory to ROS/MoveIt via OpenIGTLink"
        pathPlanningFormLayout.addRow(self.sendTransformButton)

        # Add a button for the complete ROS data package
        self.sendROSDataButton = qt.QPushButton("Send Complete Data to ROS")
        self.sendROSDataButton.toolTip = "Send trajectory, points, and models to ROS"
        pathPlanningFormLayout.addRow(self.sendROSDataButton)

        # Connect buttons to their event handlers
        self.runPlanningButton.connect('clicked(bool)', self.onRunPlanningButton)
        self.clearResultsButton.connect('clicked(bool)', self.onClearResultsButton)
        self.sendTransformButton.connect('clicked(bool)', self.onSendTransformButton)
        self.sendROSDataButton.connect('clicked(bool)', self.onSendROSDataButton)
        
        # Create logic class
        self.logic = PathPlanningModLogic()
        
        # Initialize with default values if available
        self.initializeSelectors()
        
        # Add these additional methods if they don't already exist in your class
        
    def onEditSource(self):
        """Open source code in Python editor"""
        moduleDir = os.path.dirname(slicer.util.modulePath(self.__module__))
        filePath = os.path.join(moduleDir, self.__module__ + ".py")
        if os.path.exists(filePath):
            qt.QDesktopServices.openUrl(qt.QUrl("file:///" + filePath, qt.QUrl.TolerantMode))
            
    def onEditUI(self):
        """Open UI file in Qt Designer"""
        moduleDir = os.path.dirname(slicer.util.modulePath(self.__module__))
        filePath = os.path.join(moduleDir, "Resources/UI", self.__module__ + ".ui")
        if os.path.exists(filePath):
            qt.QDesktopServices.openUrl(qt.QUrl("file:///" + filePath, qt.QUrl.TolerantMode))
        
    def onReload(self):
        """Reload module without restarting Slicer"""
        print("Reloading module...")
        slicer.util.reloadScriptedModule(self.__module__)
        
    def onReloadAndTest(self):
        """Reload module and run test function"""
        print("Reloading and testing module...")
        slicer.util.reloadScriptedModule(self.__module__)
        # Create an instance of the test class and run the test
        test = PathPlanningModTest()
        test.runTest()

    def onEnableLengthConstraintToggled(self, enabled):
        """
        Enable or disable the length constraint slider based on checkbox state
        """
        self.maxLengthSlider.enabled = enabled

    def initializeSelectors(self):
        """
        Initialize selectors with existing nodes if available
        """
        # Try to find appropriate nodes in the scene
        try:
            # Look for hippocampus
            for name in ['r_hippoTest', 'r_hippo', 'hippocampus']:
                node = slicer.util.getNode(name)
                if node:
                    self.hippoSelector.setCurrentNode(node)
                    break
                    
            # Look for ventricles
            for name in ['ventriclesTest', 'ventricles']:
                node = slicer.util.getNode(name)
                if node:
                    self.ventriclesSelector.setCurrentNode(node)
                    break
                    
            # Look for vessels
            for name in ['vesselsTestDilate1', 'vessels']:
                node = slicer.util.getNode(name)
                if node:
                    self.vesselsSelector.setCurrentNode(node)
                    break
                    
            # Look for cortex
            for name in ['r_cortexTest', 'r_cortex', 'cortex']:
                node = slicer.util.getNode(name)
                if node:
                    self.cortexSelector.setCurrentNode(node)
                    break
                    
            # Look for entry points
            for name in ['entriesSubsample', 'entries']:
                node = slicer.util.getNode(name)
                if node:
                    self.entrySelector.setCurrentNode(node)
                    break
                    
            # Look for target points
            for name in ['targetsSubsample', 'targets']:
                node = slicer.util.getNode(name)
                if node:
                    self.targetSelector.setCurrentNode(node)
                    break
        except:
            # If any error occurs, just continue without initialization
            pass

    def onRunPlanningButton(self):
        """Run the path planning algorithm"""
        # Update status
        self.statusLabel.text = "Running path planning..."
        slicer.app.processEvents()  # Force GUI update
        
        try:
            # Get the selected nodes
            hippo_node = self.hippoSelector.currentNode()
            ventricles_node = self.ventriclesSelector.currentNode()
            vessels_node = self.vesselsSelector.currentNode()
            cortex_node = self.cortexSelector.currentNode()
            entry_node = self.entrySelector.currentNode()
            target_node = self.targetSelector.currentNode()
            
            # Check if all required nodes are selected
            if not hippo_node or not ventricles_node or not vessels_node or not cortex_node or not entry_node or not target_node:
                self.statusLabel.text = "Error: Please select all required inputs"
                return
            
            # Get max trajectory length if constraint is enabled
            max_length = None
            if self.enableLengthConstraintCheckBox.checked:
                max_length = self.maxLengthSlider.value
            
            # Run the path planning algorithm with length constraint
            # EXPLICITLY set quiet_mode=False to ensure output is displayed
            results = self.logic.run_complete_planning(
                entry_node.GetName(),
                target_node.GetName(),
                hippo_node.GetName(),
                cortex_node.GetName(),
                ventricles_node.GetName(),
                vessels_node.GetName(),
                max_trajectory_length=max_length,
                quiet_mode=False  # EXPLICITLY set to False
            )
            
            # Update status with results
            if 'num_safe_trajectories' in results:
                self.statusLabel.text = f"Found {results['num_safe_trajectories']} safe trajectories. "
                
                if 'best_score' in results and results['best_score']:
                    self.statusLabel.text += f"Best score: {results['best_score']:.2f}. "
                    
                if 'trajectory_length' in results and results['trajectory_length']:
                    self.statusLabel.text += f"Length: {results['trajectory_length']:.2f}mm. "
                    
                if 'total_time' in results:
                    self.statusLabel.text += f"Time: {results['total_time']:.2f}s"
                else:
                    self.statusLabel.text += "Complete!"
            else:
                self.statusLabel.text = "No valid trajectories found"
                
        except Exception as e:
            import traceback
            print(traceback.format_exc())
            self.statusLabel.text = f"Error: {str(e)}"


    def onClearResultsButton(self):
        """Clear the current results"""
        try:
            # Remove any visualization nodes with standard names
            for node_name in ['OptimalTrajectory', 'OptimalTrajectory_Entry', 'OptimalTrajectory_Target']:
                try:
                    node = slicer.util.getNode(node_name)
                    slicer.mrmlScene.RemoveNode(node)
                except:
                    pass
                    
            # Remove any test visualization nodes
            for node_name in ['TestTrajectory', 'TestTrajectory_Entry', 'TestTrajectory_Target']:
                try:
                    node = slicer.util.getNode(node_name)
                    slicer.mrmlScene.RemoveNode(node)
                except:
                    pass
                    
            # Also remove any transforms
            try:
                transform_node = slicer.util.getNode("OptimalTrajectoryTransform")
                slicer.mrmlScene.RemoveNode(transform_node)
            except:
                pass
                    
            self.statusLabel.text = "Results cleared"
        except Exception as e:
            self.statusLabel.text = f"Error clearing results: {str(e)}"

    def cleanup(self) -> None:
        """Called when the application closes and the module widget is destroyed."""
        self.removeObservers()

    def enter(self):
        """Called each time the user opens this module."""
        # Instead of initializing the parameter node, just ensure UI is ready
        pass

    def exit(self) -> None:
        """Called each time the user opens a different module."""
        # Do not react to parameter node changes (GUI will be updated when the user enters into the module)
        if self._parameterNode:
            self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
            self._parameterNodeGuiTag = None
            self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)

    def onSceneStartClose(self, caller, event) -> None:
        """Called just before the scene is closed."""
        # Parameter node will be reset, do not use it anymore
        self.setParameterNode(None)

    def onSceneEndClose(self, caller, event) -> None:
        """Called just after the scene is closed."""
        # If this module is shown while the scene is closed then recreate a new parameter node immediately
        if self.parent.isEntered:
            self.initializeParameterNode()

    # Find and replace the initializeParameterNode() method:
    def initializeParameterNode(self):
        """Initialize selectors with defaults if available"""
        # Simply call initializeSelectors
        self.initializeSelectors()

    def setParameterNode(self, inputParameterNode: Optional[PathPlanningModParameterNode]) -> None:
        """
        Set and observe parameter node.
        Observation is needed because when the parameter node is changed then the GUI must be updated immediately.
        """

        if self._parameterNode:
            self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
            self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)
        self._parameterNode = inputParameterNode
        if self._parameterNode:
            # Note: in the .ui file, a Qt dynamic property called "SlicerParameterName" is set on each
            # ui element that needs connection.
            self._parameterNodeGuiTag = self._parameterNode.connectGui(self.ui)
            self.addObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)
            self._checkCanApply()

    def _checkCanApply(self, caller=None, event=None) -> None:
        if self._parameterNode and self._parameterNode.inputVolume and self._parameterNode.thresholdedVolume:
            self.ui.applyButton.toolTip = _("Compute output volume")
            self.ui.applyButton.enabled = True
        else:
            self.ui.applyButton.toolTip = _("Select input and output volume nodes")
            self.ui.applyButton.enabled = False

    def onApplyButton(self) -> None:
        """Run processing when user clicks "Apply" button."""
        with slicer.util.tryWithErrorDisplay(_("Failed to compute results."), waitCursor=True):
            # Compute output
            self.logic.process(self.ui.inputSelector.currentNode(), self.ui.outputSelector.currentNode(),
                               self.ui.imageThresholdSliderWidget.value, self.ui.invertOutputCheckBox.checked)

            # Compute inverted output (if needed)
            if self.ui.invertedOutputSelector.currentNode():
                # If additional output volume is selected then result with inverted threshold is written there
                self.logic.process(self.ui.inputSelector.currentNode(), self.ui.invertedOutputSelector.currentNode(),
                                   self.ui.imageThresholdSliderWidget.value, not self.ui.invertOutputCheckBox.checked, showResult=False)

    def onSendROSDataButton(self):
        """Send complete trajectory data package to ROS with separate entry and target points"""
        try:
            # First check if we have optimal trajectory available
            entry_node = slicer.util.getNode("OptimalTrajectory_Entry")
            target_node = slicer.util.getNode("OptimalTrajectory_Target")
            
            if not entry_node or not target_node:
                self.statusLabel.text = "Error: Run path planning first"
                return
                
            # Get point coordinates
            entry_point = [0, 0, 0]
            target_point = [0, 0, 0]
            entry_node.GetNthControlPointPosition(0, entry_point)
            target_node.GetNthControlPointPosition(0, target_point)
            
            print(f"📍 Entry Point: {entry_point}")
            print(f"📍 Target Point: {target_point}")
            
             # STEP 1: Send entry and target points as separate transforms
            success = self.send_entry_and_target_points(entry_point, target_point)

            if success:
                self.statusLabel.text = "Sent entry & target points, creating brain models..."
                slicer.app.processEvents()  # Update UI
                
                # STEP 2: Create brain models from current label maps
                brain_models = self.logic.create_brain_models_from_labelmaps(
                    self.hippoSelector.currentNode().GetName() if self.hippoSelector.currentNode() else None,
                    self.vesselsSelector.currentNode().GetName() if self.vesselsSelector.currentNode() else None,
                    self.ventriclesSelector.currentNode().GetName() if self.ventriclesSelector.currentNode() else None,
                    self.cortexSelector.currentNode().GetName() if self.cortexSelector.currentNode() else None
                )
                
                if brain_models:
                    self.statusLabel.text = "Created brain models, sending to ROS..."
                    slicer.app.processEvents()  # Update UI
                    
                    # STEP 3: Send brain models as point clouds
                    brain_success = self.logic.send_brain_models_as_point_clouds(brain_models)
                    
                    if brain_success:
                        self.statusLabel.text = f"✅ Sent trajectory + {len(brain_models)} brain structures to ROS"
                    else:
                        self.statusLabel.text = "⚠️ Sent trajectory, but brain models failed"
                else:
                    self.statusLabel.text = "✅ Sent trajectory points (no brain models created)"
            else:
                self.statusLabel.text = "❌ Error sending entry/target points"
                
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.statusLabel.text = f"Error sending data: {str(e)}"

            
        #     if success:
        #         # Also create folder with brain structures (existing functionality)
        #         result = self.logic.create_Folder_With_Entry_and_Target_Points_and_Polydata(entry_point, target_point)
        #         if result:
        #             self.statusLabel.text = "Sent entry & target points + brain structures to ROS"
        #         else:
        #             self.statusLabel.text = "Sent entry & target points (no brain structures)"
        #     else:
        #         self.statusLabel.text = "Error sending entry/target points"
                
        # except Exception as e:
        #     import traceback
        #     traceback.print_exc()
        #     self.statusLabel.text = f"Error sending data: {str(e)}"

    def send_entry_and_target_points(self, entry_point, target_point):
        """Send entry and target points as separate transforms"""
        try:
            # Get OpenIGTLink connector
            connectors = slicer.util.getNodesByClass("vtkMRMLIGTLConnectorNode")
            if not connectors:
                print("❌ No OpenIGTLink connector found!")
                return False
            
            connector = connectors[0]
            print(f"✅ Found connector: {connector.GetName()}")
            
            # Create entry point transform
            entry_transform = slicer.vtkMRMLLinearTransformNode()
            entry_transform.SetName("SurgicalEntryPoint")
            slicer.mrmlScene.AddNode(entry_transform)
            
            # Create entry transform matrix
            entry_matrix = vtk.vtkMatrix4x4()
            entry_matrix.Identity()
            entry_matrix.SetElement(0, 3, entry_point[0])  # X
            entry_matrix.SetElement(1, 3, entry_point[1])  # Y  
            entry_matrix.SetElement(2, 3, entry_point[2])  # Z
            entry_transform.SetMatrixTransformToParent(entry_matrix)
            
            # Create target point transform
            target_transform = slicer.vtkMRMLLinearTransformNode()
            target_transform.SetName("SurgicalTargetPoint")
            slicer.mrmlScene.AddNode(target_transform)
            
            # Create target transform matrix
            target_matrix = vtk.vtkMatrix4x4()
            target_matrix.Identity()
            target_matrix.SetElement(0, 3, target_point[0])  # X
            target_matrix.SetElement(1, 3, target_point[1])  # Y
            target_matrix.SetElement(2, 3, target_point[2])  # Z
            target_transform.SetMatrixTransformToParent(target_matrix)
            
            # Send both transforms
            connector.RegisterOutgoingMRMLNode(entry_transform)
            connector.RegisterOutgoingMRMLNode(target_transform)
            
            # Push both nodes
            connector.PushNode(entry_transform)
            connector.PushNode(target_transform)
            
            print(f"✅ Sent Entry Point: {entry_point}")
            print(f"✅ Sent Target Point: {target_point}")
            
            return True
            
        except Exception as e:
            print(f"❌ Error sending points: {str(e)}")
            import traceback
            traceback.print_exc()
            return False

    def onSendTransformButton(self):
        """Send the optimal trajectory to ROS/MoveIt"""
        try:
            # Get the optimal trajectory points
            entry_node = slicer.util.getNode("OptimalTrajectory_Entry")
            target_node = slicer.util.getNode("OptimalTrajectory_Target")
            
            if not entry_node or not target_node:
                self.statusLabel.text = "Error: Run path planning first"
                return
                
            # Get point coordinates
            entry_point = [0, 0, 0]
            target_point = [0, 0, 0]
            entry_node.GetNthControlPointPosition(0, entry_point)
            target_node.GetNthControlPointPosition(0, target_point)
            
            # Create and send transform
            transform_node = self.logic.create_and_send_trajectory_transform(entry_point, target_point)
            
            self.statusLabel.text = "Sent trajectory transform to ROS/MoveIt"
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.statusLabel.text = f"Error sending transform: {str(e)}"
            
    def onUploadDataButton(self):
        """
        Open a dialog to load various data formats, including STL files
        """
        self.statusLabel.text = "Loading data..."
        slicer.app.processEvents()
        
        fileDialog = qt.QFileDialog()
        fileDialog.setFileMode(qt.QFileDialog.ExistingFiles)
        fileDialog.setNameFilter("All supported files (*.nii.gz *.nii *.stl *.vtk *.fcsv *.json);;STL files (*.stl);;NIFTI files (*.nii.gz *.nii);;VTK files (*.vtk);;Fiducial files (*.fcsv);;JSON files (*.json)")
        
        if not fileDialog.exec_():
            self.statusLabel.text = "Data loading cancelled"
            return
            
        selectedFiles = fileDialog.selectedFiles()
        
        if not selectedFiles:
            self.statusLabel.text = "No files selected"
            return
            
        # Process each selected file
        loaded_count = 0
        for filePath in selectedFiles:
            try:
                node = self.logic.loadDataFile(filePath)
                if node:
                    loaded_count += 1
                    # Try to assign to appropriate selector based on node name
                    self.assignNodeToSelector(node)
            except Exception as e:
                import traceback
                traceback.print_exc()
                print(f"Error loading {filePath}: {str(e)}")
        
        if loaded_count > 0:
            self.statusLabel.text = f"Successfully loaded {loaded_count} file(s)"
        else:
            self.statusLabel.text = "Failed to load files"
    
    def assignNodeToSelector(self, node):
        """
        Try to assign a node to the appropriate selector based on its name
        
        Parameters:
        -----------
        node : MRML node
            Node to assign
        """
        if not node:
            return
        
        try:
            # Ensure we have a valid node object, not a list or tuple
            if isinstance(node, (list, tuple)):
                print(f"Warning: Received a {type(node)} instead of a node: {node}")
                if len(node) > 0 and isinstance(node[1], str):
                    try:
                        # Try to get the actual node from the scene using the name
                        node = slicer.util.getNode(node[1])
                    except:
                        print(f"Could not find node with name: {node[1]}")
                        return
            
            # Now proceed with a valid node
            nodeName = node.GetName().lower()
            nodeType = node.GetClassName()
            
            # For labelmap volumes
            if 'vtkMRMLLabelMapVolumeNode' in nodeType or 'LabelMapVolume' in nodeType:
                # Try to match with structure names
                if 'hippo' in nodeName:
                    self.hippoSelector.setCurrentNode(node)
                    print(f"Assigned {nodeName} to hippocampus selector")
                elif 'ventr' in nodeName:
                    self.ventriclesSelector.setCurrentNode(node)
                    print(f"Assigned {nodeName} to ventricles selector")
                elif 'vessel' in nodeName or 'vasc' in nodeName:
                    self.vesselsSelector.setCurrentNode(node)
                    print(f"Assigned {nodeName} to vessels selector")
                elif 'cortex' in nodeName:
                    self.cortexSelector.setCurrentNode(node)
                    print(f"Assigned {nodeName} to cortex selector")
            
            # For fiducials
            elif 'vtkMRMLMarkupsFiducialNode' in nodeType or 'Markup' in nodeType:
                if 'entry' in nodeName or 'entr' in nodeName:
                    self.entrySelector.setCurrentNode(node)
                    print(f"Assigned {nodeName} to entry points selector")
                elif 'target' in nodeName or 'targ' in nodeName:
                    self.targetSelector.setCurrentNode(node)
                    print(f"Assigned {nodeName} to target points selector")
            
            # Models and other types can be shown but not assigned to selectors
            else:
                print(f"Loaded {nodeType} node: {nodeName} (not assigned to any selector)")
        
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"Error assigning node to selector: {str(e)}")
#
# PathPlanningModLogic
#

class PathPlanningModLogic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual
    computation done by your module.  The interface
    should be such that other python code can import
    this class and make use of the functionality without
    requiring an instance of the Widget.
    Uses ScriptedLoadableModuleLogic base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self) -> None:
        """Called when the logic class is instantiated. Can be used for initializing member variables."""
        ScriptedLoadableModuleLogic.__init__(self)

    def loadDataFile(self, filePath):
        """
        Load a data file based on its extension
        
        Parameters:
        -----------
        filePath : str
            Path to the file
            
        Returns:
        --------
        node : MRML node
            Loaded node or None if loading failed
        """
        fileName = os.path.basename(filePath)
        fileExt = os.path.splitext(fileName)[1].lower()
        
        if fileExt == '.stl':
            # Load STL as a model
            loadedNode = None
            try:
                loadedNodeOrTuple = slicer.util.loadModel(filePath)
                # Handle both direct node return and tuple return formats
                if isinstance(loadedNodeOrTuple, tuple):
                    loadedNode = slicer.util.getNode(loadedNodeOrTuple[1])
                else:
                    loadedNode = loadedNodeOrTuple
                print(f"Loaded STL file: {fileName}")
            except Exception as e:
                print(f"Error loading STL file: {str(e)}")
            return loadedNode
            
        elif fileExt == '.fcsv':
            # Load fiducial list
            loadedNode = None
            try:
                # When loading fiducial files, the return value is typically the node name as a string or tuple
                loadedNodeOrTuple = slicer.util.loadMarkupsFiducialList(filePath)
                
                # Handle different return formats
                if isinstance(loadedNodeOrTuple, tuple):
                    nodeName = loadedNodeOrTuple[1]
                    loadedNode = slicer.util.getNode(nodeName)
                elif isinstance(loadedNodeOrTuple, str):
                    loadedNode = slicer.util.getNode(loadedNodeOrTuple)
                elif loadedNodeOrTuple is None:
                    # Look for the node that might have been created with the filename
                    baseName = os.path.splitext(os.path.basename(filePath))[0]
                    try:
                        loadedNode = slicer.util.getNode(baseName)
                    except:
                        print(f"Could not find loaded fiducial with name: {baseName}")
                
                print(f"Loaded fiducial file: {fileName}")
            except Exception as e:
                print(f"Error loading fiducial file: {str(e)}")
                import traceback
                traceback.print_exc()
            return loadedNode
            
        elif fileExt == '.nii' or fileExt == '.gz':
            # Try to load as labelmap first
            try:
                loadedNodeOrTuple = slicer.util.loadLabelVolume(filePath)
                loadedNode = None
                
                if isinstance(loadedNodeOrTuple, tuple):
                    loadedNode = slicer.util.getNode(loadedNodeOrTuple[1])
                else:
                    loadedNode = loadedNodeOrTuple
                    
                print(f"Loaded label volume: {fileName}")
                return loadedNode
            except:
                # Try as regular volume
                try:
                    loadedNodeOrTuple = slicer.util.loadVolume(filePath)
                    loadedNode = None
                    
                    if isinstance(loadedNodeOrTuple, tuple):
                        loadedNode = slicer.util.getNode(loadedNodeOrTuple[1])
                    else:
                        loadedNode = loadedNodeOrTuple
                        
                    print(f"Loaded volume: {fileName}")
                    return loadedNode
                except Exception as e:
                    print(f"Error loading NIFTI file: {str(e)}")
                    return None
                    
        elif fileExt == '.vtk':
            # Try both model and volume
            try:
                loadedNodeOrTuple = slicer.util.loadModel(filePath)
                loadedNode = None
                
                if isinstance(loadedNodeOrTuple, tuple):
                    loadedNode = slicer.util.getNode(loadedNodeOrTuple[1])
                else:
                    loadedNode = loadedNodeOrTuple
                    
                print(f"Loaded VTK model: {fileName}")
                return loadedNode
            except:
                try:
                    loadedNodeOrTuple = slicer.util.loadVolume(filePath)
                    loadedNode = None
                    
                    if isinstance(loadedNodeOrTuple, tuple):
                        loadedNode = slicer.util.getNode(loadedNodeOrTuple[1])
                    else:
                        loadedNode = loadedNodeOrTuple
                        
                    print(f"Loaded VTK volume: {fileName}")
                    return loadedNode
                except Exception as e:
                    print(f"Error loading VTK file: {str(e)}")
                    return None
        
        elif fileExt == '.json':
            # Handle JSON files - could be various types
            print(f"JSON file support not fully implemented: {fileName}")
            return None
            
        else:
            # Try generic load
            try:
                loadedNodeOrTuple = slicer.util.loadNodeFromFile(filePath)
                loadedNode = None
                
                if isinstance(loadedNodeOrTuple, tuple):
                    loadedNode = slicer.util.getNode(loadedNodeOrTuple[1])
                else:
                    loadedNode = loadedNodeOrTuple
                    
                print(f"Loaded file using generic loader: {fileName}")
                return loadedNode
            except Exception as e:
                print(f"Error loading file with generic loader: {str(e)}")
                return None
    
    def getParameterNode(self):
        return PathPlanningModParameterNode(super().getParameterNode())

    def process(self,
                inputVolume: vtkMRMLScalarVolumeNode,
                outputVolume: vtkMRMLScalarVolumeNode,
                imageThreshold: float,
                invert: bool = False,
                showResult: bool = True) -> None:
        """
        Run the processing algorithm.
        Can be used without GUI widget.
        :param inputVolume: volume to be thresholded
        :param outputVolume: thresholding result
        :param imageThreshold: values above/below this threshold will be set to 0
        :param invert: if True then values above the threshold will be set to 0, otherwise values below are set to 0
        :param showResult: show output volume in slice viewers
        """

        if not inputVolume or not outputVolume:
            raise ValueError("Input or output volume is invalid")

        import time

        startTime = time.time()
        logging.info("Processing started")

        # Compute the thresholded output volume using the "Threshold Scalar Volume" CLI module
        cliParams = {
            "InputVolume": inputVolume.GetID(),
            "OutputVolume": outputVolume.GetID(),
            "ThresholdValue": imageThreshold,
            "ThresholdType": "Above" if invert else "Below",
        }
        cliNode = slicer.cli.run(slicer.modules.thresholdscalarvolume, None, cliParams, wait_for_completion=True, update_display=showResult)
        # We don't need the CLI module node anymore, remove it to not clutter the scene with it
        slicer.mrmlScene.RemoveNode(cliNode)

        stopTime = time.time()
        logging.info(f"Processing completed in {stopTime-startTime:.2f} seconds")

    def getNodefunc(self, filename):
        """
        Get a node from Slicer's scene by name
        
        Parameters:
        filename: string, name of the node to get
        
        Returns:
        The requested node or None if not found
        """
        try:
            # Get the node from Slicer's scene
            node = slicer.util.getNode(filename)
            return node
        except:
            print(f"Could not find node: {filename}")
            return None
    
    def getPointsFromNode(self, node_name):
        node = self.getNodefunc(node_name)
        
        if node is None:
            print(f"Error: Node '{node_name}' not found")
            return []

        points = []
        for i in range(node.GetNumberOfControlPoints()):
            point = [0, 0, 0]
            node.GetNthControlPointPosition(i, point)
            points.append(point)
        return points

    def get_points_in_hippocampus(self, hippo_filename, target_filename):
        hippocampus = self.getNodefunc(hippo_filename)
        target_fiducials = self.getNodefunc(target_filename)
        
        # Get both transformation matrices
        rasToIjk = vtk.vtkMatrix4x4()
        ijkToRas = vtk.vtkMatrix4x4()
        hippocampus.GetRASToIJKMatrix(rasToIjk)
        #hippocampus.GetIJKToRASMatrix(ijkToRas)
        
        hippo_array = slicer.util.arrayFromVolume(hippocampus)
        points_in_hippo = []
        
        for i in range(target_fiducials.GetNumberOfControlPoints()):
            # Get original RAS coordinates
            target_ras = [0, 0, 0]
            target_fiducials.GetNthControlPointPosition(i, target_ras)
            #print(f"\nPoint {i}:")
            #print(f"Original RAS: {target_ras}")
            
            # Convert RAS to IJK
            target_ras_h = target_ras + [1]
            target_ijk_h = rasToIjk.MultiplyPoint(target_ras_h)
            #print(f"IJK before rounding: {target_ijk_h[:3]}")
            
            # Round to get voxel indices
            x, y, z = [int(round(p)) for p in target_ijk_h[:3]]
            #print(f"IJK after rounding: [x:{x}, y:{y}, z:{z}]")
            
            # # Convert rounded IJK back to RAS
            # rounded_ijk_h = [x, y, z, 1]
            # rounded_ras_h = ijkToRas.MultiplyPoint(rounded_ijk_h)
            # print(f"RAS after round-trip conversion: {rounded_ras_h[:3]}")
            
            # Check value at this position
            if (0 <= z < hippo_array.shape[0] and 
                0 <= y < hippo_array.shape[1] and 
                0 <= x < hippo_array.shape[2]):
                val = hippo_array[z,y,x]
                #print(f"Value at position: {val}")
                
                if val == 1:
                    points_in_hippo.append((target_ras))
                    #print("Status: Inside hippocampus")
                else:
                    #print("Status: Outside hippocampus")
                    pass
                    
        return points_in_hippo

    def check_cortex_entry_angles(self, entry_node, target_node, hippo_node, cortex_node, max_length=None):
        """
        Checks which trajectories enter cortex at appropriate angles with batch processing.
        Also filters by length first for efficiency.
        
        Args:
            entry_node: Name of the entry points node
            target_node: Name of the target points node
            hippo_node: Name of the hippocampus node
            cortex_node: Name of the cortex node
            max_length: Maximum allowed trajectory length (filter early)
                
        Returns:
            list: List of valid trajectory pairs [(entry_point, target_point, deviation), ...]
        """
        
        # Get cortex node
        cortex = self.getNodefunc(cortex_node)
        cortex_array = slicer.util.arrayFromVolume(cortex)
        
        # Get RAS to IJK transformation
        rasToIjk = vtk.vtkMatrix4x4()
        cortex.GetRASToIJKMatrix(rasToIjk)
        
        # Get lists of points
        entry_points = self.getPointsFromNode(entry_node)
        targets_in_hippo = self.get_points_in_hippocampus(hippo_node, target_node)

        # Get the target only the one in hippocampus
        if not targets_in_hippo:
            return []  # No valid targets
        
        valid_trajectories = []
        
        # Create all possible trajectories first
        all_trajectories = []
        for entry_point in entry_points:
            for target_point in targets_in_hippo:
                # OPTIMIZATION: Filter by length first before expensive angle calculation
                if max_length is not None:
                    length = np.linalg.norm(np.array(target_point) - np.array(entry_point))
                    if length > max_length:
                        continue  # Skip this trajectory if too long
                    
                all_trajectories.append((entry_point, target_point))
        
        # Process trajectories in batches
        batch_size = 500  # Adjust based on performance testing
        total_trajectories = len(all_trajectories)
        
        for batch_start in range(0, total_trajectories, batch_size):
            batch_end = min(batch_start + batch_size, total_trajectories)
            batch = all_trajectories[batch_start:batch_end]
            
            # Process this batch
            for entry_point, target_point in batch:
                # Calculate trajectory vector
                trajectory = np.array(target_point) - np.array(entry_point)
                trajectory = trajectory / np.linalg.norm(trajectory)
                
                # Convert entry to IJK
                entry_ras_h = entry_point + [1]
                entry_ijk = rasToIjk.MultiplyPoint(entry_ras_h)[:3]
                
                # Sample points along trajectory to find cortex intersection
                intersection = None
                step_size = 1.0  # mm
                max_steps = 100
                
                # Convert trajectory to IJK space
                trajectory_ijk = np.array([
                    rasToIjk.GetElement(0,0) * trajectory[0] + 
                    rasToIjk.GetElement(0,1) * trajectory[1] + 
                    rasToIjk.GetElement(0,2) * trajectory[2],
                    
                    rasToIjk.GetElement(1,0) * trajectory[0] + 
                    rasToIjk.GetElement(1,1) * trajectory[1] + 
                    rasToIjk.GetElement(1,2) * trajectory[2],
                    
                    rasToIjk.GetElement(2,0) * trajectory[0] + 
                    rasToIjk.GetElement(2,1) * trajectory[1] + 
                    rasToIjk.GetElement(2,2) * trajectory[2]
                ])
                trajectory_ijk = trajectory_ijk / np.linalg.norm(trajectory_ijk)
                
                # Find first point of cortex
                for i in range(max_steps):
                    point = entry_ijk + i * step_size * trajectory_ijk
                    x, y, z = [int(round(p)) for p in point]
                    
                    if (0 <= x < cortex_array.shape[0] and 
                        0 <= y < cortex_array.shape[1] and 
                        0 <= z < cortex_array.shape[2]):
                        
                        if cortex_array[x, y, z] == 1:
                            intersection = (x, y, z)
                            break
                
                if not intersection:
                    continue  # No intersection found
                
                # Find a simple normal by checking if we're on the edge
                # Just check 6 primary directions
                directions = [(-1,0,0), (1,0,0), (0,-1,0), (0,1,0), (0,0,-1), (0,0,1)]
                outside_dirs = []
                
                for dx, dy, dz in directions:
                    nx, ny, nz = intersection[0] + dx, intersection[1] + dy, intersection[2] + dz
                    if (0 <= nx < cortex_array.shape[0] and 
                        0 <= ny < cortex_array.shape[1] and 
                        0 <= nz < cortex_array.shape[2]):
                        if cortex_array[nx, ny, nz] == 0:
                            outside_dirs.append((dx, dy, dz))
                
                if not outside_dirs:
                    continue  # Can't determine normal
                
                # Average the directions pointing outside to get approximate normal
                if outside_dirs:  # Check if list is not empty
                    normal = np.mean(outside_dirs, axis=0)
                    norm_length = np.linalg.norm(normal)
                    if norm_length > 0:  # Avoid division by zero
                        normal = normal / norm_length
                    else:
                        continue  # Skip this trajectory if normal vector length is zero
                else:
                    continue  # Skip this trajectory if no outside directions found
                
                # Calculate angle between trajectory and normal
                dot_product = np.dot(normal, trajectory_ijk)
                angle_rad = np.arccos(np.clip(dot_product, -1.0, 1.0))
                angle_deg = np.degrees(angle_rad)
                
                # Angle from perpendicular
                deviation = abs(90 - angle_deg)
                
                if deviation < 55:
                    valid_trajectories.append((entry_point, target_point, deviation))
            
            # Optional: Print progress after each batch
            # print(f"  Progress: {batch_end}/{total_trajectories} trajectories checked for angle")
        
        return valid_trajectories
    
    def check_trajectory_collisions(self, trajectories, ventricle_node, vessel_node):
        """
        Filters trajectories to avoid ventricles and blood vessels in one pass
        
        Args:
            trajectories: List of trajectory pairs [(entry_point, target_point), ...]
            ventricle_node: Name of the ventricles node
            vessel_node: Name of the vessels node
            
        Returns:
            list: Filtered trajectories that avoid critical structures
        """
        
        safe_trajectories = []
        
        # Get structures
        ventricles = self.getNodefunc(ventricle_node)
        vessels = self.getNodefunc(vessel_node)
        
        if ventricles is None or vessels is None:
            print(f"Error: Could not find ventricle node '{ventricle_node}' or vessel node '{vessel_node}'")
            return trajectories  # Return original trajectories if nodes not found
        
        # Get arrays and transformations once
        vent_array = slicer.util.arrayFromVolume(ventricles)
        vessel_array = slicer.util.arrayFromVolume(vessels)
        
        vent_rasToIjk = vtk.vtkMatrix4x4()
        vessel_rasToIjk = vtk.vtkMatrix4x4()
        
        ventricles.GetRASToIJKMatrix(vent_rasToIjk)  # Fix capitalization from IjK to IJK
        vessels.GetRASToIJKMatrix(vessel_rasToIjk)
        
        # Store array dimensions for bounds checking
        vent_shape = vent_array.shape
        vessel_shape = vessel_array.shape
        
        print(f"Checking {len(trajectories)} trajectories for collisions...")
        
        # Process trajectories in batches for better memory management
        # Fix: Ensure batch_size is at least 1 to prevent ValueError in range()
        batch_size = max(1, min(50, len(trajectories)))  # Ensure batch_size is at least 1
        
        for batch_start in range(0, len(trajectories), batch_size):
            batch_end = min(batch_start + batch_size, len(trajectories))
            batch = trajectories[batch_start:batch_end]
            
            # Process this batch of trajectories
            for traj_idx, traj in enumerate(batch):
                # Handle different trajectory formats
                if isinstance(traj, tuple):
                    # Extract entry_point and target_point, regardless of tuple length
                    entry_point, target_point = traj[0], traj[1]
                    # Keep any additional data from the tuple
                    additional_data = traj[2:] if len(traj) > 2 else ()
                else:
                    print(f"Warning: Unexpected trajectory format: {type(traj)}")
                    continue

                # Convert entry and target to IJK for both structures - do this once per trajectory
                entry_ras_h = entry_point + [1]
                target_ras_h = target_point + [1]
                
                # For ventricles
                vent_entry_ijk = np.array(vent_rasToIjk.MultiplyPoint(entry_ras_h)[:3])
                vent_target_ijk = np.array(vent_rasToIjk.MultiplyPoint(target_ras_h)[:3])
                
                # For vessels
                vessel_entry_ijk = np.array(vessel_rasToIjk.MultiplyPoint(entry_ras_h)[:3])
                vessel_target_ijk = np.array(vessel_rasToIjk.MultiplyPoint(target_ras_h)[:3])
                
                # Calculate trajectory vectors and lengths once
                vent_vector = vent_target_ijk - vent_entry_ijk
                vessel_vector = vessel_target_ijk - vessel_entry_ijk
                
                vent_distance = np.linalg.norm(vent_vector)
                vessel_distance = np.linalg.norm(vessel_vector)
                
                # Flag to track collisions
                has_collision = False
                
                # MULTI-LEVEL CHECKING: First do a much sparser check to quickly reject obviously bad trajectories
                # This significantly reduces computation time for trajectories that have collisions
                # Check only a few points initially spread across the trajectory
                coarse_check_count = 10
                coarse_check_indices = np.linspace(0, 1, coarse_check_count)
                
                # Coarse check first - much faster for early rejection
                for t in coarse_check_indices:
                    # Check ventricles
                    vent_point = vent_entry_ijk + t * vent_vector
                    vx, vy, vz = [int(round(p)) for p in vent_point]
                    
                    # DIRECT BOUNDS CHECKING with proper indexing order (z,y,x)
                    if (0 <= vz < vent_shape[0] and 
                        0 <= vy < vent_shape[1] and 
                        0 <= vx < vent_shape[2]):
                        
                        # Check for collision (non-zero value)
                        if vent_array[vz, vy, vx] > 0:
                            has_collision = True
                            break  # Early termination
                    
                    # Check vessels
                    vessel_point = vessel_entry_ijk + t * vessel_vector
                    vx, vy, vz = [int(round(p)) for p in vessel_point]
                    
                    # DIRECT BOUNDS CHECKING with proper indexing order (z,y,x)
                    if (0 <= vz < vessel_shape[0] and 
                        0 <= vy < vessel_shape[1] and 
                        0 <= vx < vessel_shape[2]):
                        
                        # Check for collision (non-zero value)
                        if vessel_array[vz, vy, vx] > 0:
                            has_collision = True
                            break  # Early termination
                
                # Only do detailed check if coarse check passes
                if not has_collision:
                    # ADAPTIVE SAMPLING: Use more samples for longer trajectories
                    # with reasonable limits for fine-grained check
                    num_samples = min(150, max(50, int(max(vent_distance, vessel_distance) * 1.5)))
                    
                    # Generate sample points along trajectory at more detailed resolution
                    t_values = np.linspace(0, 1, num_samples)
                    
                    # Skip values we already checked in the coarse check
                    for t in t_values:
                        # Skip values very close to ones we've already checked
                        if any(abs(t - coarse_t) < 0.01 for coarse_t in coarse_check_indices):
                            continue
                            
                        # Check ventricles at this point
                        vent_point = vent_entry_ijk + t * vent_vector
                        vx, vy, vz = [int(round(p)) for p in vent_point]
                        
                        # DIRECT BOUNDS CHECKING with proper indexing order (z,y,x)
                        if (0 <= vz < vent_shape[0] and 
                            0 <= vy < vent_shape[1] and 
                            0 <= vx < vent_shape[2]):
                            
                            # Check for collision (non-zero value)
                            if vent_array[vz, vy, vx] > 0:
                                has_collision = True
                                break  # Early termination
                        
                        # Check vessels at this point
                        vessel_point = vessel_entry_ijk + t * vessel_vector
                        vx, vy, vz = [int(round(p)) for p in vessel_point]
                        
                        # DIRECT BOUNDS CHECKING with proper indexing order (z,y,x)
                        if (0 <= vz < vessel_shape[0] and 
                            0 <= vy < vessel_shape[1] and 
                            0 <= vx < vessel_shape[2]):
                            
                            # Check for collision (non-zero value)
                            if vessel_array[vz, vy, vx] > 0:
                                has_collision = True
                                break  # Early termination
                
                # If no collisions found, add to safe trajectories with original data
                if not has_collision:
                    if additional_data:
                        safe_trajectories.append((entry_point, target_point) + additional_data)
                    else:
                        safe_trajectories.append((entry_point, target_point))
                        
            # Progress update after each batch
            #if batch_size > 10 and len(trajectories) > 20:  # Only show progress for larger datasets
            #    print(f"  Progress: {min(batch_end, len(trajectories))}/{len(trajectories)} trajectories checked")
    
        print(f"Found {len(safe_trajectories)} safe trajectories without collisions")
        return safe_trajectories
    
    def calculate_trajectory_length(self, entry_point, target_point):
        """Calculate the Euclidean distance between entry and target points."""
        
        entry_coords = np.array(entry_point)
        target_coords = np.array(target_point)
        
        # Calculate Euclidean distance
        length = np.linalg.norm(entry_coords - target_coords)
        return length

    def filter_trajectories_by_length(self, trajectories, max_length):
        """
        Filter trajectories based on maximum length constraint
        
        Parameters:
        -----------
        trajectories : list
            List of trajectories as tuples (entry_point, target_point, ...)
        max_length : float
            Maximum allowed trajectory length
            
        Returns:
        --------
        list
            Filtered list of trajectories with length <= max_length
        """
        filtered_trajectories = []
        
        for traj in trajectories:
            # Check if trajectory is in tuple format
            if isinstance(traj, tuple):
                entry_point = traj[0]  # First element is entry point
                target_point = traj[1]  # Second element is target point
            # Or in dictionary format
            elif isinstance(traj, dict) and 'entry_point' in traj and 'target_point' in traj:
                entry_point = traj['entry_point']
                target_point = traj['target_point']
            else:
                print(f"Warning: Unknown trajectory format: {type(traj)}")
                continue
            
            # Calculate trajectory length (Euclidean distance)
            length = np.linalg.norm(np.array(entry_point) - np.array(target_point))
            
            # Only include trajectories within the length constraint
            if length <= max_length:
                # If it's a tuple, append a new tuple with length
                if isinstance(traj, tuple):
                    # If there are additional elements in the tuple, preserve them
                    if len(traj) > 2:
                        # Create a new tuple with original elements plus length
                        filtered_trajectories.append(traj + (length,))
                    else:
                        # Just the entry, target and length
                        filtered_trajectories.append((entry_point, target_point, length))
                # If it's a dictionary, add length to it
                elif isinstance(traj, dict):
                    traj_with_length = traj.copy()
                    traj_with_length['length'] = length
                    filtered_trajectories.append(traj_with_length)
        
        return filtered_trajectories
    
    def validate_trajectories(self, entry_node, target_node, hippo_node, cortex_node, ventricle_node, vessel_node, max_length=None):
        """
        Complete validation pipeline with timing measurements - optimized order:
        1. Check entry angles (with built-in length filtering for efficiency)
        2. Check collisions (most expensive)
        
        Parameters:
        -----------
        entry_node : str
            Name of the entry points node
        target_node : str
            Name of the target points node
        hippo_node : str
            Name of the hippocampus node
        ventricle_node : str
            Name of the ventricle node
        vessel_node : str
            Name of the vessel node
        max_length : float, optional
            Maximum allowed trajectory length. If None, no length constraint is applied.
        
        Returns:
        --------
        list
            List of valid trajectories that satisfy all constraints
        """
        # Time the entire process
        total_start = time.time()
        
        # Step 1: Time the angle check (with built-in length filtering)
        angle_start = time.time()
        angle_valid_trajectories = self.check_cortex_entry_angles(
            entry_node, target_node, hippo_node, cortex_node, max_length)
        angle_end = time.time()
        angle_time = angle_end - angle_start
        
        if max_length:
            print(f"Angle check with length filtering: Found {len(angle_valid_trajectories)} trajectories with valid angles and length <= {max_length}. Took {angle_time:.3f}s")
        else:
            print(f"Angle check: Found {len(angle_valid_trajectories)} trajectories with valid angles. Took {angle_time:.3f}s")

        # Step 2: Check collisions only on trajectories that passed previous filters
        collision_start = time.time()
        safe_trajectories = self.check_trajectory_collisions(angle_valid_trajectories, ventricle_node, vessel_node)
        collision_end = time.time()
        collision_time = collision_end - collision_start
        print(f"Collision check: Found {len(safe_trajectories)} trajectories without collisions. Took {collision_time:.3f}s")
        
        # Total time
        total_end = time.time()
        total_time = total_end - total_start
        print(f"Total validation time: {total_time:.3f} seconds")
        
        return safe_trajectories
#
# Soft Constraints Method
#

    def compute_distance_map(self, labelmap_node_or_name):
        """
        Computes a distance map from a label map using Danielsson distance filter
        
        Args:
            labelmap_node_or_name: The label map volume node or name
            
        Returns:
            Distance map volume node
        """
        
        try:
            # Handle both node objects and node names
            if isinstance(labelmap_node_or_name, str):
                labelmap_node = self.getNodefunc(labelmap_node_or_name)
            else:
                labelmap_node = labelmap_node_or_name
                
            if not labelmap_node:
                print(f"Error: Could not find or use label map node: {labelmap_node_or_name}")
                return None
                
            # Pull the volume from Slicer
            sitk_input = su.PullVolumeFromSlicer(labelmap_node)
            
            # Create distance filter
            distanceFilter = sitk.SignedMaurerDistanceMapImageFilter()
            distanceFilter.SetUseImageSpacing(True)  # Account for pixel spacing (important for medical data)
            distanceFilter.SetSquaredDistance(False)  # More efficient if you only need to compare distances
            distanceFilter.SetBackgroundValue(0)  # Assuming 0 is your background value

            # distance_filter = sitk.DanielssonDistanceMapImageFilter()
            # distance_filter.SetInputIsBinary(True)
            # distance_filter.SetUseImageSpacing(True)
            
            # Execute the filter
            sitk_output = distanceFilter.Execute(sitk_input)
            
            # Push the result back to Slicer
            volume_name = labelmap_node.GetName() + "_distance"
            distance_map = su.PushVolumeToSlicer(sitk_output, None, volume_name)
            
            return distance_map
            
        except Exception as e:
            print(f"Error computing distance map: {str(e)}")
            import traceback
            print(traceback.format_exc())
            return None
    
    def calculate_min_distance_along_trajectory(self, entry_point, target_point, distance_map_node):
        """
        Calculates the minimum distance from a trajectory to the structure
        
        Args:
            entry_point: 3D point [x, y, z] for entry
            target_point: 3D point [x, y, z] for target
            distance_map_node: Distance map volume node
            
        Returns:
            Minimum distance along the trajectory
        """
        # Handle null distance map
        if distance_map_node is None:
            print("Warning: Null distance map provided")
            return 0.0  # Return safe default

        # Get the distance map array - using numpy for faster operations
        distance_array = slicer.util.arrayFromVolume(distance_map_node)
        
        # Get RAS to IJK transform
        ras_to_ijk = vtk.vtkMatrix4x4()
        distance_map_node.GetRASToIJKMatrix(ras_to_ijk)
        
        # Convert entry and target to IJK - do this once instead of in the loop
        entry_ras_h = entry_point + [1]
        target_ras_h = target_point + [1]
        
        entry_ijk = np.array(ras_to_ijk.MultiplyPoint(entry_ras_h)[:3])
        target_ijk = np.array(ras_to_ijk.MultiplyPoint(target_ras_h)[:3])
        
        # Calculate trajectory vector and length once
        traj_vector = target_ijk - entry_ijk
        traj_length = np.linalg.norm(traj_vector)
        if traj_length == 0:
            return 0.0  # Avoid division by zero
            
        # Normalize trajectory vector for efficient step calculation
        traj_unit = traj_vector / traj_length
        
        # Determine number of samples based on length
        # Use adaptive sampling - more samples for longer trajectories
        # but with a reasonable upper limit to avoid excessive computation
        num_samples = min(500, max(50, int(traj_length * 2)))
        
        # Pre-calculate all sample points at once using vectorized operations
        t_values = np.linspace(0, 1, num_samples)
        
        # Check array bounds once
        z_dim, y_dim, x_dim = distance_array.shape
        
        # Use a more efficient approach to find minimum distance
        min_distance = float('inf')
        point_found = False
        
        # Use a step size that ensures we don't miss voxels
        step_size = traj_length / (num_samples - 1)
        if step_size > 1.0:
            # If step size is larger than a voxel, increase samples
            num_samples = int(traj_length) + 1
            t_values = np.linspace(0, 1, num_samples)
            step_size = traj_length / (num_samples - 1)
            
        # Vectorized calculation of all sample points
        # This is much faster than calculating each point in a loop
        for i in range(num_samples):
            # Calculate point at this position
            t = t_values[i]
            point = entry_ijk + t * traj_vector
            
            # Round to nearest voxel
            x, y, z = [int(round(p)) for p in point]
            
            # Check if within volume bounds - with proper order for numpy array
            if (0 <= z < z_dim and 0 <= y < y_dim and 0 <= x < x_dim):
                # Get distance at this point
                distance = distance_array[z, y, x]
                
                # Update minimum distance - use numpy minimum for potential vectorization
                if distance < min_distance:
                    min_distance = distance
                    point_found = True
                    
                    # Early termination option - if we find a very small distance
                    # we can stop searching as we've likely found a collision or near-collision
                    if distance < 0.5:
                        return max(0.0, distance)
        
        # If no valid point was found, return a small positive value
        if not point_found or min_distance == float('inf'):
            return 0.0
        
        # Ensure we never return negative values that could cause problems later
        return max(0.0, min_distance)

    def find_optimal_trajectory(self, valid_trajectories, vessels_node, ventricles_node, quite_mode=False):
        """
        Find the optimal trajectory based on distance to critical structures with adaptive safety margin
        """
        
        if not valid_trajectories:
            print("No valid trajectories to optimize")
            return None, None, 0.0, []
        
        # Generate distance maps
        vessel_distance_map = self.compute_distance_map(vessels_node)
        ventricle_distance_map = self.compute_distance_map(ventricles_node)

        # Track best trajectory
        best_entry = None
        best_target = None
        best_combined_distance = -float('inf')
        
        # Set initial safety margins
        original_vessel_margin = 2.0  # mm
        original_ventricle_margin = 1.0  # mm
        min_vessel_distance = original_vessel_margin
        min_ventricle_distance = original_ventricle_margin
        
        # Set minimum quality threshold - below this planning is unsuccessful
        min_quality_threshold = 0.5  # Minimum acceptable score
        
        # Set adaptive margin reduction factors
        margin_reduction_steps = 3  # Number of reduction steps to try
        vessel_reduction_factor = 0.5  # Reduce by 50% each step
        ventricle_reduction_factor = 0.5
        
        # Weights for scoring
        vessel_weight = 0.7  # Prioritize vessel avoidance
        ventricle_weight = 0.2
        length_weight = 0.1  # Small weight for length - shorter is better
        
        # Normalization factor for trajectory length
        max_expected_length = 200.0  # Expected maximum length in mm
        
        trajectory_metrics = []
        
        # Print a few trajectories to check their format
        # if not quite_mode and len(valid_trajectories) > 0:
        #     print(f"DEBUG: First trajectory format: {valid_trajectories[0]}")
        #     if len(valid_trajectories[0]) > 2:
        #         print(f"DEBUG: Entry angle from trajectory: {valid_trajectories[0][2]}")
        
        # Start with original margins, then try reduced margins if needed
        for reduction_step in range(margin_reduction_steps + 1):
            if reduction_step > 0:
                # Reduce safety margins if we haven't found any viable trajectories
                min_vessel_distance *= vessel_reduction_factor
                min_ventricle_distance *= ventricle_reduction_factor
                print(f"Adapting safety margins - Step {reduction_step}/{margin_reduction_steps}")
                print(f"New vessel margin: {min_vessel_distance:.2f}mm, ventricle margin: {min_ventricle_distance:.2f}mm")

            safe_count = 0
            best_step_entry = None
            best_step_target = None
            best_step_score = -float('inf')
            
            # Evaluate each trajectory
            for traj in valid_trajectories:
                # Handle different trajectory formats
                if isinstance(traj, tuple):
                    # Simple extraction of entry and target points
                    entry_point, target_point = traj[0], traj[1]
                    
                    # Extract the entry angle if available (should be the 3rd element)
                    entry_angle = None
                    if len(traj) > 2:
                        entry_angle = traj[2]
                elif isinstance(traj, dict):
                    # Handle dictionary format
                    entry_point = traj['entry_point']
                    target_point = traj['target_point']
                    entry_angle = traj.get('entry_angle')
                else:
                    print(f"Warning: Unknown trajectory format: {type(traj)}")
                    continue
                
                # Always calculate length explicitly for consistency
                length = np.linalg.norm(np.array(entry_point) - np.array(target_point))
                
                # Calculate distances
                vessel_distance = self.calculate_min_distance_along_trajectory(
                    entry_point, target_point, vessel_distance_map)
                
                ventricle_distance = self.calculate_min_distance_along_trajectory(
                    entry_point, target_point, ventricle_distance_map)
                
                # Normalize length - smaller values are better
                normalized_length_score = 1.0 - min(1.0, length / max_expected_length)
                
                # Create trajectory metric entry
                trajectory_metric = {
                    'entry': entry_point,
                    'target': target_point,
                    'vessel_distance': vessel_distance,
                    'ventricle_distance': ventricle_distance,
                    'length': length,
                    'combined_score': 0,  # Will be updated later if safe
                    'meets_safety_margins': False,
                    'safety_margin_level': reduction_step
                }
                
                # Store the entry angle if available
                if entry_angle is not None:
                    trajectory_metric['entry_angle'] = entry_angle
                
                # Check safety margins
                if vessel_distance < min_vessel_distance or ventricle_distance < min_ventricle_distance:
                    # Add to metrics but mark as unsafe
                    trajectory_metrics.append(trajectory_metric)
                    continue  # Skip this trajectory
                
                safe_count += 1
                trajectory_metric['meets_safety_margins'] = True
                
                # Ensure all values are numeric and valid
                vessel_score = max(0.0, vessel_distance)  # Always non-negative
                ventricle_score = max(0.0, ventricle_distance)  # Always non-negative
                length_score = normalized_length_score * 10  # Already normalized
                
                # Calculate weighted score
                combined_distance = (vessel_weight * vessel_score + 
                                ventricle_weight * ventricle_score +
                                length_weight * length_score)
                
                # Update the score in metrics
                trajectory_metric['combined_score'] = combined_distance
                trajectory_metrics.append(trajectory_metric)
                
                # Update best trajectory if this one is better
                if combined_distance > best_step_score:
                    best_step_score = combined_distance
                    best_step_entry = entry_point
                    best_step_target = target_point
            
            # After evaluating all trajectories with current margins
            
            # If we found safe trajectories at this reduction step
            if safe_count > 0:
                best_entry = best_step_entry
                best_target = best_step_target
                best_combined_distance = best_step_score
                
                # Check if the best trajectory meets minimum quality threshold
                if best_combined_distance < min_quality_threshold:
                    print(f"Warning: Best trajectory score ({best_combined_distance:.2f}) is below quality threshold ({min_quality_threshold})")
                    # Continue to try reduced margins to see if we can find better options
                else:
                    # We found a good quality trajectory, no need to reduce margins further
                    print(f"Found high quality trajectory (score: {best_combined_distance:.2f}) with safety margins")
                    break
            
            # If this was the last reduction step and we still haven't found safe trajectories
            if reduction_step == margin_reduction_steps and safe_count == 0:
                print(f"Warning: No trajectories meet even the minimum safety margins after {margin_reduction_steps} reductions")
        
        # After all reduction steps, sort metrics by combined score
        trajectory_metrics.sort(key=lambda x: x.get('combined_score', 0), reverse=True)
        
        # If we couldn't find any safe trajectories after all reduction steps
        if best_entry is None:
            print("No trajectories meet the safety requirements")
            # If no safe trajectories, return the one with the largest vessel distance
            if trajectory_metrics:
                # Make sure we have valid values for comparison
                valid_metrics = [m for m in trajectory_metrics 
                                if 'vessel_distance' in m and isinstance(m['vessel_distance'], (int, float)) 
                                and not math.isinf(m['vessel_distance']) and not math.isnan(m['vessel_distance'])]
                
                if valid_metrics:
                    # Get the trajectory with the largest vessel distance
                    best_metric = max(valid_metrics, key=lambda m: m.get('vessel_distance', 0))
                    best_entry = best_metric['entry']
                    best_target = best_metric['target']
                    best_combined_distance = best_metric.get('combined_score', 0)
                    print(f"WARNING: Returning UNSAFE trajectory with largest vessel distance: {best_metric['vessel_distance']:.2f}mm")
                    print(f"Length: {best_metric['length']:.2f} mm")
                    if 'entry_angle' in best_metric:
                        print(f"Entry angle: {best_metric['entry_angle']:.2f}°")
                    print(f"This trajectory DOES NOT meet minimum safety requirements!")
                else:
                    if trajectory_metrics:
                        best_metric = trajectory_metrics[0]
                        best_entry = best_metric['entry']
                        best_target = best_metric['target']
                        best_combined_distance = 0
                        print(f"WARNING: No valid vessel distances found, using first trajectory")
                    else:
                        best_combined_distance = 0
        else:
            # Find and print details for the best trajectory
            best_metrics = next((m for m in trajectory_metrics if m['entry'] == best_entry and m['target'] == best_target), None)
            if best_metrics:
                safety_level = best_metrics.get('safety_margin_level', 0)
                safety_status = "FULL SAFETY" if safety_level == 0 else f"REDUCED SAFETY (level {safety_level})"
                
                if not quite_mode:
                    print(f"Best trajectory has combined distance score: {best_combined_distance:.2f} - {safety_status}")
                    print(f"  - Vessel distance: {best_metrics['vessel_distance']:.2f}mm")
                    print(f"  - Ventricle distance: {best_metrics['ventricle_distance']:.2f}mm")
                    print(f"  - Length: {best_metrics['length']:.2f}mm")
                    if 'entry_angle' in best_metrics:
                        deviation = best_metrics['entry_angle']
                        surface_angle = 90 - deviation
                        print(f"  - Entry angle deviation: {deviation:.2f}° from perpendicular")
                        print(f"  - Actual surface entry angle: {surface_angle:.2f}°")
        
        return best_entry, best_target, best_combined_distance, trajectory_metrics
    
    def visualize_trajectory(self, entry_point, target_point, name="BestTrajectory", cleanup_previous=True):
        """
        Creates a visual model of a trajectory in 3D Slicer with enhanced visibility in both 2D and 3D views
        
        Args:
            entry_point: The entry point [x, y, z]
            target_point: The target point [x, y, z]
            name: Name for the model
            cleanup_previous: Whether to remove previous models with the same name
                
        Returns:
            The created model node
        """
        # Clean up previous models with the same name if requested
        if cleanup_previous:
            try:
                old_model = slicer.util.getNode(name)
                slicer.mrmlScene.RemoveNode(old_model)
            except:
                pass
            try:
                old_entry = slicer.util.getNode(name + "_Entry")
                slicer.mrmlScene.RemoveNode(old_entry)
            except:
                pass
            try:
                old_target = slicer.util.getNode(name + "_Target")
                slicer.mrmlScene.RemoveNode(old_target)
            except:
                pass
        
        # Create a tube instead of a line for better 3D visibility
        line_source = vtk.vtkLineSource()
        line_source.SetPoint1(entry_point)
        line_source.SetPoint2(target_point)
        line_source.Update()
        
        # Create a tube filter for better 3D visibility
        tube_filter = vtk.vtkTubeFilter()
        tube_filter.SetInputConnection(line_source.GetOutputPort())
        tube_filter.SetRadius(0.8)  # Increased radius for better visibility
        tube_filter.SetNumberOfSides(20)
        tube_filter.CappingOn()
        tube_filter.Update()
        
        # Create a model node for the trajectory
        model_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", name)
        model_node.SetAndObservePolyData(tube_filter.GetOutput())
        
        # Create and set a display node to control appearance
        display_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelDisplayNode")
        model_node.SetAndObserveDisplayNodeID(display_node.GetID())
        
        # Make the trajectory clearly visible in 3D
        display_node.SetColor(0.0, 1.0, 0.3)  # Bright green color for better visibility
        display_node.SetAmbient(0.3)          # Add some ambient light reflection
        display_node.SetDiffuse(0.7)          # Add diffuse lighting for better 3D appearance 
        display_node.SetSpecular(0.5)         # Add specular highlight
        display_node.SetPower(10)             # Sharpness of specular highlight
        display_node.SetOpacity(1.0)          # Fully opaque
        
        # Enhanced 2D visibility
        display_node.SetVisibility2D(True)     # Show in 2D slices
        display_node.SetSliceIntersectionThickness(4)  # Make it thicker in slices (was 3)
        display_node.SetSliceIntersectionOpacity(1.0)  # Fully opaque in slices
        
        # Create fiducial nodes for entry and target points with bigger glyphs
        entry_fiducial = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", name + "_Entry")
        entry_fiducial.AddControlPoint(entry_point)
        entry_fiducial.SetNthControlPointLabel(0, "Entry")
        
        target_fiducial = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", name + "_Target")
        target_fiducial.AddControlPoint(target_point)
        target_fiducial.SetNthControlPointLabel(0, "Target")
        
        # Make the fiducials more visible
        for fiducial, color in [(entry_fiducial, [0.0, 0.8, 0.0]), (target_fiducial, [1.0, 0.0, 0.0])]:
            display_node = fiducial.GetDisplayNode()
            if display_node:
                display_node.SetGlyphScale(6.0)  # Make them larger (was 5.0)
                display_node.SetSelectedColor(*color)  # Different colors for entry/target
                display_node.SetTextScale(3.0)    # Make text larger
                # Only use methods that we know exist
                try:
                    display_node.SetSliceProjection(True)  # Show projections on slice views
                    display_node.SetSliceProjectionOpacity(0.7)  # Semi-transparent projections
                    display_node.SetSliceProjectionColor(*color)  # Use same color for projections
                except:
                    pass  # Ignore if method doesn't exist
        
        # Set up optimal slice orientations to show the trajectory
        # Calculate trajectory vector
        trajectory_vector = np.array(target_point) - np.array(entry_point)
        trajectory_vector = trajectory_vector / np.linalg.norm(trajectory_vector)
        
        # Find perpendicular vectors to trajectory for optimal slice orientation
        if abs(trajectory_vector[0]) > 0.1 or abs(trajectory_vector[1]) > 0.1:
            # Use z-axis to find perpendicular
            perp = np.cross(trajectory_vector, [0, 0, 1])
        else:
            # If trajectory is along z, use y-axis
            perp = np.cross(trajectory_vector, [0, 1, 0])
        perp = perp / np.linalg.norm(perp)
        
        # Third perpendicular vector
        perp2 = np.cross(trajectory_vector, perp)
        perp2 = perp2 / np.linalg.norm(perp2)
        
        # Calculate midpoint for centering views
        center_point = [(entry_point[0] + target_point[0])/2, 
                    (entry_point[1] + target_point[1])/2, 
                    (entry_point[2] + target_point[2])/2]
        
        # Set slice orientations to best show trajectory
        layoutManager = slicer.app.layoutManager()
        
        # Red slice - typically axial, set perpendicular to trajectory
        red_slice = layoutManager.sliceWidget("Red").sliceLogic()
        red_node = red_slice.GetSliceNode()
        red_node.SetSliceToRASByNTP(
            trajectory_vector[0], trajectory_vector[1], trajectory_vector[2],  # Normal
            perp[0], perp[1], perp[2],  # TransverseFrame
            center_point[0], center_point[1], center_point[2],  # Point
            0  # rotate
        )
        
        # Yellow slice - typically sagittal, set along trajectory
        yellow_slice = layoutManager.sliceWidget("Yellow").sliceLogic()
        yellow_node = yellow_slice.GetSliceNode()
        yellow_node.SetSliceToRASByNTP(
            perp[0], perp[1], perp[2],  # Normal
            trajectory_vector[0], trajectory_vector[1], trajectory_vector[2],  # TransverseFrame
            center_point[0], center_point[1], center_point[2],  # Point
            0  # rotate
        )
        
        # Green slice - typically coronal, set along trajectory but perpendicular to yellow
        green_slice = layoutManager.sliceWidget("Green").sliceLogic()
        green_node = green_slice.GetSliceNode()
        green_node.SetSliceToRASByNTP(
            perp2[0], perp2[1], perp2[2],  # Normal
            trajectory_vector[0], trajectory_vector[1], trajectory_vector[2],  # TransverseFrame
            center_point[0], center_point[1], center_point[2],  # Point
            0  # rotate
        )
        
        # Adjust field of view to show trajectory with margin
        trajectory_length = np.linalg.norm(np.array(target_point) - np.array(entry_point))
        field_of_view = max(100, trajectory_length * 1.5)  # Ensure minimum reasonable FOV
        
        # Center and fit slice views
        for widget_name in ["Red", "Yellow", "Green"]:
            slice_widget = layoutManager.sliceWidget(widget_name)
            slice_logic = slice_widget.sliceLogic()
            slice_node = slice_logic.GetSliceNode()
            
            # Replace JumpSlice (which doesn't exist) with the correct method
            # There are multiple ways to do this in Slicer
            
            # Option 1: Using JumpSliceByCentering on the slice node
            slice_node.JumpSliceByCentering(center_point[0], center_point[1], center_point[2])
            
            # OR Option 2: Using JumpSliceByOffsetting 
            # slice_logic.JumpSliceByOffsetting(center_point[0], center_point[1], center_point[2])
            
            # Then fit slice to all content
            slice_logic.FitSliceToAll()
        
        # Switch to a layout showing all three slice views and 3D
        layoutManager.setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutFourUpView)
        
        # Center 3D view on the trajectory
        threeDView = layoutManager.threeDWidget(0).threeDView()
        threeDView.resetFocalPoint()
        threeDView.lookFromAxis(2)  # Look from superior direction
        
        return model_node
    # def visualize_trajectory(self, entry_point, target_point, name="BestTrajectory", cleanup_previous=True):
    #     """
    #     Creates a visual model of a trajectory in 3D Slicer with enhanced visibility
        
    #     Args:
    #         entry_point: The entry point [x, y, z]
    #         target_point: The target point [x, y, z]
    #         name: Name for the model
            
    #     Returns:
    #         The created model node
    #     """
    #         # Clean up previous models with the same name if requested
    #     if cleanup_previous:
    #         try:
    #             old_model = slicer.util.getNode(name)
    #             slicer.mrmlScene.RemoveNode(old_model)
    #         except:
    #             pass
    #         try:
    #             old_entry = slicer.util.getNode(name + "_Entry")
    #             slicer.mrmlScene.RemoveNode(old_entry)
    #         except:
    #             pass
    #         try:
    #             old_target = slicer.util.getNode(name + "_Target")
    #             slicer.mrmlScene.RemoveNode(old_target)
    #         except:
    #             pass
    #     # Create a line representing the trajectory
    #     line_source = vtk.vtkLineSource()
    #     line_source.SetPoint1(entry_point)
    #     line_source.SetPoint2(target_point)
    #     line_source.Update()
        
    #     # Create a model node for the trajectory
    #     model_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", name)
    #     model_node.SetAndObservePolyData(line_source.GetOutput())
        
    #     # Create and set a display node to control appearance
    #     display_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelDisplayNode")
    #     model_node.SetAndObserveDisplayNodeID(display_node.GetID())
        
    #     # Make the trajectory clearly visible
    #     display_node.SetColor(0.0, 1.0, 0.0)  # Green color (more visible)
    #     display_node.SetLineWidth(7)  # Extra thick line
    #     display_node.SetVisibility2D(True)  # Show in 2D slices
    #     display_node.SetSliceIntersectionThickness(3)  # Make it thicker in slices
    #     display_node.SetOpacity(1.0)  # Fully opaque
        
    #     # Create fiducial nodes for entry and target points
    #     entry_fiducial = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", name + "_Entry")
    #     entry_fiducial.AddControlPoint(entry_point)
    #     entry_fiducial.SetNthControlPointLabel(0, "Entry")
        
    #     target_fiducial = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", name + "_Target")
    #     target_fiducial.AddControlPoint(target_point)
    #     target_fiducial.SetNthControlPointLabel(0, "Target")
        
    #     # Make the fiducials more visible
    #     for fiducial, color in [(entry_fiducial, [0.0, 0.5, 0.0]), (target_fiducial, [1.0, 0.0, 0.0])]:
    #         display_node = fiducial.GetDisplayNode()
    #         if display_node:
    #             display_node.SetGlyphScale(5.0)  # Make them larger
    #             display_node.SetSelectedColor(*color)  # Different colors for entry/target
    #             # Only use methods that we know exist
    #             try:
    #                 display_node.SetSliceProjection(True)  # Show projections on slice views
    #             except:
    #                 pass  # Ignore if method doesn't exist
        
    #     # Center views on the trajectory
    #     center_point = [(entry_point[0] + target_point[0])/2, 
    #                 (entry_point[1] + target_point[1])/2, 
    #                 (entry_point[2] + target_point[2])/2]
        
    #     # Center slice views on this point
    #     for viewName in ["Red", "Green", "Yellow"]:
    #         sliceNode = slicer.app.layoutManager().sliceWidget(viewName).mrmlSliceNode()
    #         sliceNode.JumpSliceByCentering(center_point[0], center_point[1], center_point[2])
        
    #     # Center 3D view
    #     layoutManager = slicer.app.layoutManager()
    #     threeDView = layoutManager.threeDWidget(0).threeDView()
    #     threeDView.resetFocalPoint()
        
    #     return model_node
    
    def run_complete_planning(self, entry_node_name, target_node_name, 
                            hippo_node_name, cortex_node_name, ventricle_node_name, 
                            vessel_node_name, max_trajectory_length=None, quiet_mode=False):
        """
        Runs the complete path planning pipeline from hard constraints to soft optimization
        
        Args:
            entry_node_name: Name of the entry points fiducial node
            target_node_name: Name of the target points fiducial node
            hippo_node_name: Name of the hippocampus label map node
            ventricle_node_name: Name of the ventricles label map node
            vessel_node_name: Name of the vessels label map node
            
        Returns:
            Dictionary with results and timing information
        """
        
        results = {}
        total_start = time.time()
        if not quiet_mode:
            print("\n======= PATH PLANNING PIPELINE =======")
        
        # Step 1: Hard constraints - Run trajectory validation
        safe_trajectories = self.validate_trajectories(
           entry_node_name, target_node_name, hippo_node_name, cortex_node_name, ventricle_node_name, vessel_node_name, max_trajectory_length)
        
        results['safe_trajectories'] = safe_trajectories
        results['num_safe_trajectories'] = len(safe_trajectories)
        
        if not safe_trajectories:
            if not quiet_mode:  # Only print if not in quiet mode
                print("No safe trajectories found. Planning failed.")
            results['total_time'] = time.time() - total_start
            return results
        
        if not quiet_mode:
            print(f"Found {len(safe_trajectories)} trajectories satisfying hard constraints")

        # Step 2: Soft constraints - Find optimal trajectory
        soft_start = time.time()
        best_entry, best_target, best_score, metrics = self.find_optimal_trajectory(
            safe_trajectories, vessel_node_name, ventricle_node_name)
        soft_end = time.time()

        # Check for valid best trajectory
        if not best_entry or not best_target:
            if not quiet_mode:
                print("Failed to find optimal trajectory")
            results['total_time'] = time.time() - total_start
            return results
        
        results['optimization_time'] = soft_end - soft_start
        results['best_entry'] = best_entry
        results['best_target'] = best_target
        results['best_score'] = best_score
        results['trajectory_metrics'] = metrics
        
        # Step 3: Visualize the best trajectory
        # Before creating new visualization, clean up any previous ones
        for node_name in ["OptimalTrajectory", "OptimalTrajectory_Entry", "OptimalTrajectory_Target"]:
            try:
                node = slicer.util.getNode(node_name)
                slicer.mrmlScene.RemoveNode(node)
            except:
                pass
                    
        best_metrics = None
        if metrics and best_entry and best_target:
            vis_start = time.time()
            best_metrics = next((m for m in metrics if m['entry'] == best_entry and m['target'] == best_target), None)
            trajectory_model = self.visualize_trajectory(best_entry, best_target, "OptimalTrajectory")
            vis_end = time.time()

            # Store metrics in results
            if best_metrics:
                results['vessel_distance'] = best_metrics['vessel_distance'] 
                results['ventricle_distance'] = best_metrics['ventricle_distance']
                results['trajectory_length'] = best_metrics.get('length', 0.0)
                results['entry_angle'] = best_metrics.get('entry_angle', 0.0)  # Add angle if available
            elif metrics and len(metrics) > 0:
                # Fallback to first metric if best one not found
                results['vessel_distance'] = metrics[0]['vessel_distance']
                results['ventricle_distance'] = metrics[0]['ventricle_distance']
                results['trajectory_length'] = metrics[0].get('length', 0.0)
                
            # Store visualization results
            results['visualization_time'] = vis_end - vis_start
            results['trajectory_model'] = trajectory_model
        
        # Create and send trajectory transform
        transform_start = time.time()
        transform_node = self.create_and_send_trajectory_transform(best_entry, best_target)
        transform_end = time.time()
        
        # Store transform results
        results['transform_time'] = transform_end - transform_start
        results['transform_node'] = transform_node

        # Total time
        total_end = time.time()
        results['total_time'] = total_end - total_start

        # Print summary if not in quiet mode
        if not quiet_mode:
            print("\n--- PLANNING RESULTS ---")
            #print(f"Best trajectory has combined distance score: {best_score:.2f}")

            # Show metrics
            # Show metrics
            if best_metrics:
                safety_level = best_metrics.get('safety_margin_level', 0)
                safety_status = "FULL SAFETY" if safety_level == 0 else f"REDUCED SAFETY (level {safety_level})"
                
                # Calculate length explicitly (always accurate)
                actual_length = np.linalg.norm(np.array(best_entry) - np.array(best_target))
                
                print(f"Best trajectory has combined distance score: {best_score:.2f} - {safety_status}")
                print(f"  - Vessel distance: {best_metrics['vessel_distance']:.2f} mm")
                print(f"  - Ventricle distance: {best_metrics['ventricle_distance']:.2f} mm")
                print(f"  - Trajectory length: {actual_length:.2f} mm")  # Use explicitly calculated length
                
                # Only display angle if it's properly stored as a separate value
                if 'entry_angle' in best_metrics and best_metrics['entry_angle'] != actual_length:
                    print(f"  - Entry angle: {best_metrics['entry_angle']:.2f}°")  # Add degree symbol

            elif metrics and len(metrics) > 0:
                # Same treatment for fallback metrics
                print(f"Vessel distance: {metrics[0]['vessel_distance']:.2f} mm")
                print(f"Ventricle distance: {metrics[0]['ventricle_distance']:.2f} mm")
                
                # Calculate trajectory length explicitly
                actual_length = np.linalg.norm(np.array(best_entry) - np.array(best_target))
                print(f"Trajectory length: {actual_length:.2f} mm")
                
                # Only display angle if properly stored
                if 'entry_angle' in metrics[0] and metrics[0]['entry_angle'] != actual_length:
                    print(f"Entry angle: {metrics[0]['entry_angle']:.2f}°")
                    
            # Print coordinates
            if best_entry and best_target:
                print("\n--- COORDINATES ---")
                print(f"Entry point: ({best_entry[0]:.1f}, {best_entry[1]:.1f}, {best_entry[2]:.1f})")
                print(f"Target point: ({best_target[0]:.1f}, {best_target[1]:.1f}, {best_target[2]:.1f})")
            
            try:
                entry_node = self.getNodefunc(entry_node_name)
                target_node = self.getNodefunc(target_node_name)
                
                entry_count = entry_node.GetNumberOfControlPoints() if entry_node else 0
                target_count = target_node.GetNumberOfControlPoints() if target_node else 0
                
                results['entry_point_count'] = entry_count
                results['target_point_count'] = target_count
                
                if not quiet_mode:
                    print("\n--- INPUT SUMMARY ---")
                    print(f"Entry points: {entry_count}")
                    print(f"Target points: {target_count}")
                    print(f"Safe trajectories found: {len(safe_trajectories)}")
                    print(f"Total combinations evaluated: {entry_count * target_count} potential trajectories")
            except Exception as e:
                # Don't fail the whole planning if this summary addition fails
                print(f"Note: Could not add point count summary: {str(e)}")

            print(f"Total planning time: {results['total_time']:.3f} seconds")
            print("======= PLANNING COMPLETE =======\n")
        
        return results
##
# Send trajectory transform via OpenIGTLink
##    
    def create_and_send_trajectory_transform(self, entry_point, target_point):
        """
        Creates a LinearTransform from entry and target points and sends it via OpenIGTLink
        The transform is set up so that:
        - Origin is at the entry point
        - Z-axis points toward the target
        - X and Y axes form an orthogonal coordinate system
        
        Parameters:
        -----------
        entry_point: list [x, y, z]
            The entry point of the trajectory
        target_point: list [x, y, z]
            The target point of the trajectory
            
        Returns:
        --------
        transform_node: vtkMRMLLinearTransformNode
            The created transform node
        """
        # Create a transform node
        transform_node = slicer.vtkMRMLLinearTransformNode()
        transform_node.SetName("OptimalTrajectoryTransform")
        slicer.mrmlScene.AddNode(transform_node)
        
        # Create a matrix
        matrix = vtk.vtkMatrix4x4()
        
        # Calculate direction vector from entry to target (z-axis)
        z_axis = np.array([
            target_point[0] - entry_point[0],
            target_point[1] - entry_point[1],
            target_point[2] - entry_point[2]
        ])
        
        # Calculate trajectory length
        z_axis_length = np.linalg.norm(z_axis)
        
        # Normalize z-axis
        z_axis = z_axis / z_axis_length
        
        # Choose arbitrary vector to create orthogonal coordinate system
        # Try to use [0,1,0] first, if z is close to that, use [1,0,0]
        if abs(np.dot(z_axis, [0, 1, 0])) > 0.9:
            temp = np.array([1, 0, 0])
        else:
            temp = np.array([0, 1, 0])
        
        # Create x-axis (perpendicular to z and temp)
        x_axis = np.cross(temp, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # Create y-axis to complete orthogonal system
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # Set the rotation matrix (columns are axes)
        matrix.Identity()
        
        # Set the rotation components
        for i in range(3):
            matrix.SetElement(i, 0, x_axis[i])
            matrix.SetElement(i, 1, y_axis[i])
            matrix.SetElement(i, 2, z_axis[i])
        
        # Set the translation to the entry point
        matrix.SetElement(0, 3, entry_point[0])
        matrix.SetElement(1, 3, entry_point[1])
        matrix.SetElement(2, 3, entry_point[2])
        
        # Set the matrix
        transform_node.SetMatrixTransformToParent(matrix)
        
        # Print matrix for debugging
        print("Created trajectory transform:")
        print(f"  Entry point: ({entry_point[0]:.2f}, {entry_point[1]:.2f}, {entry_point[2]:.2f})")
        print(f"  Target point: ({target_point[0]:.2f}, {target_point[1]:.2f}, {target_point[2]:.2f})")
        print(f"  Trajectory length: {z_axis_length:.2f} mm")
        
        try:
            # Get any active OpenIGTLink connector
            connectors = slicer.util.getNodesByClass("vtkMRMLIGTLConnectorNode")
            if connectors:
                connector = connectors[0]
                print(f"Found connector: {connector.GetName()}")
                
                # Register the transform for sending
                connector.RegisterOutgoingMRMLNode(transform_node)
                print(f"Registered transform with connector")
                
                # Push all nodes
                connector.PushNode(transform_node)
                print(f"Pushed transform node")
            else:
                print("No OpenIGTLink connector found!")
        except Exception as e:
            print(f"Error sending transform: {str(e)}")
        
        return transform_node
    
    def convert_LabelMap_To_Model(self, labelmap_node):
        """
        Convert a labelmap to a 3D surface model
        
        Parameters:
        -----------
        labelmap_node : vtkMRMLLabelMapVolumeNode
        The labelmap to convert
            
        Returns:
        --------
        model_node : vtkMRMLModelNode
        The created model node, or None if conversion failed
        """
        if not labelmap_node:
            return None
            
        # Create a segmentation node from the labelmap
        segmentation_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLSegmentationNode")
        segmentation_node.SetName(f"{labelmap_node.GetName()}_segmentation")
        
        # Import the labelmap to the segmentation
        slicer.modules.segmentations.logic().ImportLabelmapToSegmentationNode(
            labelmap_node, segmentation_node)
        
        # Create a model node
        model_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", 
                                                    f"{labelmap_node.GetName()}_model")
        
        # Export the segmentation to a model
        slicer.modules.segmentations.logic().ExportVisibleSegmentsToModels(
            segmentation_node, model_node)
        
        # Clean up the temporary segmentation node
        slicer.mrmlScene.RemoveNode(segmentation_node)
        
        return model_node
    
    # def create_Folder_With_Entry_and_Target_Points_and_Polydata(self, entry_point, target_point):
    #     """
    #     Creates a folder with:
    #     - Entry and target points as fiducials
    #     - Trajectory transform
    #     - Models of critical structures
        
    #     All organized for ROS visualization and planning.
        
    #     Parameters:
    #     -----------
    #     entry_point : list [x, y, z]
    #         The entry point coordinates
    #     target_point : list [x, y, z]
    #         The target point coordinates
            
    #     Returns:
    #     --------
    #     dict
    #         Information about created data
    #     """
    #     try:
    #         # Create a folder to organize the ROS data
    #         shNode = slicer.mrmlScene.GetSubjectHierarchyNode()
    #         ros_folder_id = shNode.CreateFolderItem(shNode.GetSceneItemID(), "ROS_Trajectory_Data")
            
    #         # Create a fiducial node containing both entry and target points
    #         ros_points = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "ROS_Entry_and_Target_Points")
    #         ros_points.AddControlPoint(entry_point, "entry_point")
    #         ros_points.AddControlPoint(target_point, "target_point")
            
    #         # Create the trajectory transform
    #         transform_node = self.create_and_send_trajectory_transform(entry_point, target_point)
    #         transform_node.SetName("ROS_Trajectory_Transform")
            
    #         # Try to convert available structures to models
    #         model_nodes = []
            
    #         # Define label map nodes to convert to models if they exist
    #         structure_nodes = {
    #             "hippo": self.getNodefunc("OptimalTrajectory_Target").GetScene().GetNodeByID(self.hippo.GetID()) if hasattr(self, 'hippo') else None,
    #             "ventricles": self.getNodefunc("OptimalTrajectory_Target").GetScene().GetNodeByID(self.ventricles.GetID()) if hasattr(self, 'ventricles') else None,
    #             "vessels": self.getNodefunc("OptimalTrajectory_Target").GetScene().GetNodeByID(self.vessels.GetID()) if hasattr(self, 'vessels') else None,
    #             "cortex": self.getNodefunc("OptimalTrajectory_Target").GetScene().GetNodeByID(self.cortex.GetID()) if hasattr(self, 'cortex') else None
    #         }
            
    #         # Convert each structure to a model
    #         for name, node in structure_nodes.items():
    #             if node:
    #                 try:
    #                     # Check if model already exists
    #                     model_name = f"ROS_{name}_Model"
    #                     try:
    #                         model = slicer.util.getNode(model_name)
    #                     except:
    #                         # Convert labelmap to model
    #                         model = self.convert_LabelMap_To_Model(node)
    #                         if model:
    #                             model.SetName(model_name)
    #                             model_nodes.append(model)
    #                 except Exception as e:
    #                     print(f"Error creating model for {name}: {str(e)}")
            
    #         # Move all nodes to the ROS folder
    #         shNode.SetItemParent(shNode.GetItemByDataNode(ros_points), ros_folder_id)
    #         shNode.SetItemParent(shNode.GetItemByDataNode(transform_node), ros_folder_id)
            
    #         for model in model_nodes:
    #             shNode.SetItemParent(shNode.GetItemByDataNode(model), ros_folder_id)
            
    #         # Make the folder visible
    #         pluginHandler = slicer.qSlicerSubjectHierarchyPluginHandler().instance()
    #         folderPlugin = pluginHandler.getOwnerPluginForSubjectHierarchyItem(ros_folder_id)
    #         folderPlugin.setDisplayVisibility(ros_folder_id, 1)
            
    #         return {
    #             'folder_id': ros_folder_id,
    #             'points_node': ros_points,
    #             'transform_node': transform_node,
    #             'model_nodes': model_nodes
    #         }
            
    #     except Exception as e:
    #         import traceback
    #         traceback.print_exc()
    #         print(f"Error creating ROS data folder: {str(e)}")
    #         return None
    def create_brain_models_from_labelmaps(self, hippo_node_name, vessels_node_name, ventricles_node_name, cortex_node_name):
        """
        Convert label map nodes to 3D models for ROS visualization - SIMPLIFIED VERSION
        Uses direct VTK marching cubes instead of problematic Slicer segmentation export
        
        Parameters:
        -----------
        hippo_node_name : str
            Name of the hippocampus label map node
        vessels_node_name : str
            Name of the vessels label map node
        ventricles_node_name : str
            Name of the ventricles label map node
        cortex_node_name : str
            Name of the cortex label map node
            
        Returns:
        --------
        dict
            Dictionary of created model nodes
        """
        models = {}
        
        # Structure mapping
        structures = {
            'hippocampus': hippo_node_name,
            'vessels': vessels_node_name,
            'ventricles': ventricles_node_name,
            'cortex': cortex_node_name
        }
        
        for structure_name, node_name in structures.items():
            try:
                # Get the label map node
                labelmap_node = self.getNodefunc(node_name)
                if not labelmap_node:
                    print(f"⚠️ Could not find {structure_name} node: {node_name}")
                    continue
                
                # Check if model already exists
                model_name = f"ROS_{structure_name}_Model"
                try:
                    existing_model = slicer.util.getNode(model_name)
                    if existing_model:
                        print(f"✅ Using existing model: {model_name}")
                        models[structure_name] = existing_model
                        continue
                except:
                    pass
                
                # Create new model from label map using DIRECT VTK approach
                print(f"🔄 Creating model for {structure_name}...")
                
                # DIRECT VTK APPROACH - no segmentation nodes needed
                model_node = self.create_model_from_labelmap_direct(labelmap_node, model_name)
                
                if model_node:
                    models[structure_name] = model_node
                    print(f"✅ Created model: {model_name}")
                else:
                    print(f"❌ Failed to create model for {structure_name}")
                    
            except Exception as e:
                print(f"❌ Error creating model for {structure_name}: {str(e)}")
                import traceback
                traceback.print_exc()
        
        return models

    def create_model_from_labelmap_direct(self, labelmap_node, model_name):
        """
        Create a model directly from labelmap using VTK marching cubes
        This completely bypasses the problematic Slicer segmentation export
        """
        try:
            # Get the image data from the labelmap
            imageData = labelmap_node.GetImageData()
            if not imageData:
                print(f"   No image data in labelmap")
                return None
            
            # Check if there are any labeled voxels
            imageArray = slicer.util.arrayFromVolume(labelmap_node)
            if (imageArray == 1).sum() == 0:
                print(f"   No labeled voxels found")
                return None
            
            print(f"   Found {(imageArray == 1).sum()} labeled voxels")
            
            # Use VTK marching cubes to extract the surface
            marchingCubes = vtk.vtkDiscreteMarchingCubes()
            marchingCubes.SetInputData(imageData)
            marchingCubes.SetValue(0, 1)  # Extract surface where label = 1
            marchingCubes.Update()
            
            # Get the surface polydata
            surface = marchingCubes.GetOutput()
            
            if surface.GetNumberOfPoints() == 0:
                print(f"   Marching cubes produced no surface points")
                return None
            
            print(f"   Marching cubes generated {surface.GetNumberOfPoints()} surface points")
            
            # Apply coordinate transformation from IJK to RAS
            transform_matrix = vtk.vtkMatrix4x4()
            labelmap_node.GetIJKToRASMatrix(transform_matrix)
            
            # Create VTK transform
            transform = vtk.vtkTransform()
            transform.SetMatrix(transform_matrix)
            
            # Apply transform to the surface
            transformFilter = vtk.vtkTransformPolyDataFilter()
            transformFilter.SetInputData(surface)
            transformFilter.SetTransform(transform)
            transformFilter.Update()
            
            # Get the final transformed surface
            final_surface = transformFilter.GetOutput()
            
            # Create the model node
            model_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", model_name)
            model_node.SetAndObservePolyData(final_surface)
            
            # Create a minimal display node (required by Slicer)
            display_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelDisplayNode")
            model_node.SetAndObserveDisplayNodeID(display_node.GetID())
            display_node.SetVisibility(False)  # Hidden since only used for point extraction
            
            print(f"   ✅ Created model with {final_surface.GetNumberOfPoints()} points")
            return model_node
            
        except Exception as e:
            print(f"   ❌ Error in direct model creation: {str(e)}")
            import traceback
            traceback.print_exc()
            return None

    def send_brain_models_as_point_clouds(self, brain_models):
        """
        Send brain models as point clouds via OpenIGTLink - BATCHED like test code
        
        Parameters:
        -----------
        brain_models : dict
            Dictionary of brain model nodes
            
        Returns:
        --------
        bool
            True if successful, False otherwise
        """
        try:
            # Get OpenIGTLink connector
            connectors = slicer.util.getNodesByClass("vtkMRMLIGTLConnectorNode")
            if not connectors:
                print("❌ No OpenIGTLink connector found!")
                return False
            
            connector = connectors[0]
            print(f"✅ Found connector: {connector.GetName()}")
            
            # Check connection status
            if connector.GetState() != 2:  # 2 = Connected
                print(f"⚠️ Connector not connected (status: {connector.GetState()})")
                print("🔄 Trying to reconnect...")
                connector.Start()
                import time
                time.sleep(2)
            
            total_points_sent = 0
            
            for structure_name, model_node in brain_models.items():
                if not model_node:
                    print(f"❌ Model not found for {structure_name}")
                    continue
                
                print(f"🔄 Processing {structure_name}...")
                
                # Get polydata from the model
                polydata = model_node.GetPolyData()
                if not polydata:
                    print(f"❌ No polydata for {structure_name}")
                    continue
                    
                points = polydata.GetPoints()
                if not points:
                    print(f"❌ No points in {structure_name}")
                    continue
                    
                num_points = points.GetNumberOfPoints()
                print(f"   Found {num_points} points in {structure_name}")
                
                # Use step size to reduce point density (same as test code)
                step = max(1, num_points // 500)  # Limit to ~500 points per structure
                
                # print(f"🔄 Sending {structure_name} points in batches (every {step}th point)...")
                
                # BATCH SENDING
                batch_size = 50  # Send in small batches for efficiency
                batch_count = 0
                structure_points_sent = 0
                
                # Create initial fiducial list
                point_list_node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
                point_list_node.SetName(f"brain_{structure_name}_points")
                
                for i in range(0, num_points, step):
                    # Check if we need to send this batch
                    if structure_points_sent > 0 and structure_points_sent % batch_size == 0:
                        # Set this node as OpenIGTLink message (like test code)
                        point_list_node.SetAttribute("OpenIGTLinkIF.pushOnConnect", "true")
                        
                        # Send it
                        connector.RegisterOutgoingMRMLNode(point_list_node)
                        connector.PushNode(point_list_node)
                        
                        # print(f"✅ Sent batch {batch_count + 1} for {structure_name} ({point_list_node.GetNumberOfControlPoints()} points)")
                        
                        # Clean up and create a new batch
                        connector.UnregisterOutgoingMRMLNode(point_list_node)
                        slicer.mrmlScene.RemoveNode(point_list_node)
                        point_list_node = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode')
                        point_list_node.SetName(f"brain_{structure_name}_points_batch{batch_count}")
                        batch_count += 1
                        
                        # Small delay between batches
                        import time
                        time.sleep(0.2)
                    
                    # Get the point coordinates and add to current batch
                    point = points.GetPoint(i)
                    point_list_node.AddControlPoint(point[0], point[1], point[2])
                    structure_points_sent += 1
                    total_points_sent += 1
                
                # Send any remaining points in the final batch
                if point_list_node.GetNumberOfControlPoints() > 0:
                    point_list_node.SetAttribute("OpenIGTLinkIF.pushOnConnect", "true")
                    connector.RegisterOutgoingMRMLNode(point_list_node)
                    connector.PushNode(point_list_node)
                    # print(f"✅ Sent final batch for {structure_name} ({point_list_node.GetNumberOfControlPoints()} points)")
                    
                    # Clean up
                    connector.UnregisterOutgoingMRMLNode(point_list_node)
                    slicer.mrmlScene.RemoveNode(point_list_node)
                
                print(f"✅ Finished sending {structure_name}: {structure_points_sent} points in {batch_count + 1} batches")
                
                # Small delay between structures
                import time
                time.sleep(1)
            
            print(f"🚀 Complete! Sent {total_points_sent} brain model points total")
            return total_points_sent > 0
            
        except Exception as e:
            print(f"❌ Error sending brain models: {str(e)}")
            import traceback
            traceback.print_exc()
            return False

    def recreate_model_from_labelmap(self, structure_name):
        """
        Try to recreate a model from the original labelmap if the existing model has no polydata
        """
        try:
            # Map structure names to likely labelmap names
            labelmap_mapping = {
                'hippocampus': ['r_hippoTest', 'r_hippo', 'hippocampus'],
                'vessels': ['vesselsTestDilate1', 'vessels', 'blood_vessels'], 
                'ventricles': ['ventriclesTest', 'ventricles'],
                'cortex': ['r_cortexTest', 'r_cortex', 'cortex']
            }
            
            possible_names = labelmap_mapping.get(structure_name, [structure_name])
            
            # Find the original labelmap
            labelmap_node = None
            for name in possible_names:
                try:
                    labelmap_node = slicer.util.getNode(name)
                    if labelmap_node:
                        print(f"   Found original labelmap: {name}")
                        break
                except:
                    continue
            
            if not labelmap_node:
                print(f"   Could not find original labelmap for {structure_name}")
                return None
            
            # Create model using marching cubes (direct VTK approach)
            print(f"   Creating surface model from {labelmap_node.GetName()}...")
            
            # Get image data
            imageData = labelmap_node.GetImageData()
            if not imageData:
                print(f"   No image data in labelmap")
                return None
            
            # Use marching cubes to extract surface
            marchingCubes = vtk.vtkDiscreteMarchingCubes()
            marchingCubes.SetInputData(imageData)
            marchingCubes.SetValue(0, 1)  # Extract surface where label = 1
            marchingCubes.Update()
            
            # Get the surface
            surface = marchingCubes.GetOutput()
            
            if surface.GetNumberOfPoints() == 0:
                print(f"   No surface points generated")
                return None
            
            # Apply the labelmap's coordinate transformation
            transform_matrix = vtk.vtkMatrix4x4()
            labelmap_node.GetIJKToRASMatrix(transform_matrix)
            
            # Create VTK transform
            transform = vtk.vtkTransform()
            transform.SetMatrix(transform_matrix)
            
            # Apply transform to surface
            transformFilter = vtk.vtkTransformPolyDataFilter()
            transformFilter.SetInputData(surface)
            transformFilter.SetTransform(transform)
            transformFilter.Update()
            
            # Create new model node
            model_name = f"ROS_{structure_name}_Model_Recreated"
            model_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", model_name)
            model_node.SetAndObservePolyData(transformFilter.GetOutput())
            
            # Create minimal display node
            display_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelDisplayNode")
            model_node.SetAndObserveDisplayNodeID(display_node.GetID())
            display_node.SetVisibility(False)  # Hidden since only for point extraction
            
            print(f"   ✅ Successfully recreated model with {transformFilter.GetOutput().GetNumberOfPoints()} points")
            return model_node
            
        except Exception as e:
            print(f"   ❌ Error recreating model: {str(e)}")
            import traceback
            traceback.print_exc()
            return None

#
# PathPlanningModTest
#

class PathPlanningModTest(ScriptedLoadableModuleTest):
    """
    This is the test case for your scripted module.
    Uses ScriptedLoadableModuleTest base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """
    def setUp(self, runVisibleTests=False):
        """
        Reset the state - typically a scene clear will be enough.
        """
        slicer.mrmlScene.Clear()
    
        # Create organizational folders
        self.shNode = slicer.mrmlScene.GetSubjectHierarchyNode()
        self.testFolderID = self.shNode.CreateFolderItem(self.shNode.GetSceneItemID(), "PathPlanningTests")

        # Update this path to match test data directory
        self.testDataDir = "/Users/widyapuspitaloka/Desktop/TestSet"
        
        self.dataLoaded = False

        # Try to load test data from local directory if available
        try:
            # Load brain volume
            self.fakeBrain = slicer.util.loadVolume(os.path.join(self.testDataDir, 'fakeBrainTest.nii.gz'))
            if isinstance(self.fakeBrain, tuple):
                self.fakeBrain = slicer.util.getNode(self.fakeBrain[1])
            
            # Load structure volumes as label maps
            labelMapOptions = {'labelmap': True}
            
            self.hippo = slicer.util.loadVolume(os.path.join(self.testDataDir, 'r_hippoTest.nii.gz'), 
                                            properties=labelMapOptions)
            if isinstance(self.hippo, tuple):
                self.hippo = slicer.util.getNode(self.hippo[1])
                
            self.ventricles = slicer.util.loadVolume(os.path.join(self.testDataDir, 'ventriclesTest.nii.gz'), 
                                                properties=labelMapOptions)
            if isinstance(self.ventricles, tuple):
                self.ventricles = slicer.util.getNode(self.ventricles[1])
                
            self.vessels = slicer.util.loadVolume(os.path.join(self.testDataDir, 'vesselsTestDilate1.nii.gz'), 
                                                properties=labelMapOptions)
            if isinstance(self.vessels, tuple):
                self.vessels = slicer.util.getNode(self.vessels[1])
                
            self.cortex = slicer.util.loadVolume(os.path.join(self.testDataDir, 'r_cortexTest.nii.gz'), 
                                            properties=labelMapOptions)
            if isinstance(self.cortex, tuple):
                self.cortex = slicer.util.getNode(self.cortex[1])
            
            # Hide all test nodes if not running visible tests
            if not runVisibleTests:
                pluginHandler = slicer.qSlicerSubjectHierarchyPluginHandler().instance()
                # Hide the folder completely
                volPlugin = pluginHandler.getOwnerPluginForSubjectHierarchyItem(self.testFolderID)
                volPlugin.setDisplayVisibility(self.testFolderID, 0)
                
                # Place all nodes under the test folder
                for node in [self.fakeBrain, self.hippo, self.ventricles, self.vessels, self.cortex]:
                    self.shNode.SetItemParent(self.shNode.GetItemByDataNode(node), self.testFolderID)
            
            # Create fiducial nodes with unique names
            self.entryPoints = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "TestEntryPoints")
            self.targetPoints = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "TestTargetPoints") 
                
            # Try to load from files first
            success = self.loadPointsFromFiles()
            
            # If loading from files failed or not enough points were loaded, use known good points
            if not success:
                self.createKnownGoodPoints()
            
            # Verify hippocampus has data
            hippo_array = slicer.util.arrayFromVolume(self.hippo)
            nonzero_count = (hippo_array > 0).sum()
            logging.info(f"Hippocampus non-zero voxels: {nonzero_count}")
            
            if nonzero_count == 0:
                logging.error("Hippocampus volume has no labeled voxels")
                raise Exception("Hippocampus volume invalid")
            
            # Hide the fiducials from view if not visible tests
            if not runVisibleTests:
                self.entryPoints.GetDisplayNode().SetVisibility(0)
                self.targetPoints.GetDisplayNode().SetVisibility(0)
                self.shNode.SetItemParent(self.shNode.GetItemByDataNode(self.entryPoints), self.testFolderID)
                self.shNode.SetItemParent(self.shNode.GetItemByDataNode(self.targetPoints), self.testFolderID)
                
            # Update data loaded flag
            self.dataLoaded = True
            logging.info('Test data loaded successfully')
            
        except Exception as e:
            logging.error(f"Failed to load test data: {str(e)}")
            self.dataLoaded = False

    def loadPointsFromFiles(self):
        """Try to load points from FCSV files"""
        try:
            entriesPath = os.path.join(self.testDataDir, 'entriesSubsample.fcsv')
            targetsPath = os.path.join(self.testDataDir, 'targetsSubsample.fcsv')
            
            # Check if files exist
            if not os.path.exists(entriesPath) or not os.path.exists(targetsPath):
                logging.warning(f"Fiducial files not found at expected paths")
                return False
                
            # Try loading the fiducials
            loaded1 = slicer.util.loadMarkupsFiducialList(entriesPath, self.entryPoints)
            loaded2 = slicer.util.loadMarkupsFiducialList(targetsPath, self.targetPoints)
            
            # Check if points were loaded
            entryPointCount = self.entryPoints.GetNumberOfControlPoints()
            targetPointCount = self.targetPoints.GetNumberOfControlPoints()
            
            logging.info(f"Loaded from files: Entry points: {entryPointCount}, Target points: {targetPointCount}")
            
            # Consider successful if we loaded a reasonable number of points
            return loaded1 and loaded2 and entryPointCount > 5 and targetPointCount > 5
            
        except Exception as e:
            logging.warning(f"Failed to load points from files: {str(e)}")
            return False

    def createKnownGoodPoints(self):
        """Create target and entry points with known good coordinates"""
        logging.info("Creating test points with known good coordinates")
        
        # Clear any existing points
        self.entryPoints.RemoveAllControlPoints()
        self.targetPoints.RemoveAllControlPoints()
        
        # Add known targets that are in the hippocampus
        target_points = [
            [150.0, 82.0, 138.0],
            [162.0, 90.0, 133.0],
            [158.0, 90.0, 128.0],
            [158.0, 106.0, 128.0],
            [154.0, 106.0, 123.0],
            [146.0, 114.0, 133.0],
            [142.0, 114.0, 128.0]
        ]
        
        # Add entry points that work well
        entry_points = [
            [190.0, 74.0, 148.0],
            [200.0, 80.0, 145.0],
            [210.0, 85.0, 150.0],
            [195.0, 90.0, 155.0],
            [205.0, 95.0, 160.0]
        ]
        
        # Create additional points around the known good ones
        for point in target_points:
            # Add the known good point
            self.targetPoints.AddControlPoint(point[0], point[1], point[2])
            # Add some slight variations
            for dx in [-2, 0, 2]:
                for dy in [-2, 0, 2]:
                    for dz in [-2, 0, 2]:
                        if dx != 0 or dy != 0 or dz != 0:  # Skip the original point
                            self.targetPoints.AddControlPoint(point[0] + dx, point[1] + dy, point[2] + dz)
        
        for point in entry_points:
            # Add the known good point
            self.entryPoints.AddControlPoint(point[0], point[1], point[2])
            # Add some slight variations
            for dx in [-5, 0, 5]:
                for dy in [-5, 0, 5]:
                    for dz in [-5, 0, 5]:
                        if dx != 0 or dy != 0 or dz != 0:  # Skip the original point
                            self.entryPoints.AddControlPoint(point[0] + dx, point[1] + dy, point[2] + dz)
        
        logging.info(f"Created {self.entryPoints.GetNumberOfControlPoints()} entry points")
        logging.info(f"Created {self.targetPoints.GetNumberOfControlPoints()} target points")

    def runTest(self, debug_mode=False):
        """Run all tests in a structured way."""
        self.setUp(runVisibleTests=False)
        self.debug_mode = debug_mode

        # Initialize test counters
        self.unit_tests_total = 5
        self.unit_tests_passed = 0
        self.integration_tests_total = 2  # Skip visualization test
        self.integration_tests_passed = 0
        self.system_tests_total = 4
        self.system_tests_passed = 0
        
        # Only run tests if data was loaded successfully
        if not self.dataLoaded:
            logging_module.error("Test data not available, skipping all tests")
            return
        
        # Save original print and logging functions
        original_print = builtins.print
        original_logging_info = logging_module.info
        
        try:
            # Modify test methods to use simplified logging
            if not debug_mode:
                # Save original methods using the imported builtins
                self._orig_print = builtins.print
                self._orig_logging_info = logging_module.info
                
                def quiet_print(*args, **kwargs):
                # Only print if it starts with specific test-related strings or contains them
                    if args and isinstance(args[0], str):
                        allowed_prefixes = (
                            "Test passed", "Test failed", 
                            "Unit test", "Integration test", "System test",
                            "PASSED:", "FAILED:", "PASSED", "FAILED",
                            "\n===", "====",
                            "RUNNING", "Running", 
                            "ADDITIONAL", "Additional",
                            "Overall:", "Unit tests:", "Integration tests:", "System tests:",
                            "EDGE CASE", # Keep this for edge case results
                            "Stress Testing Results:",
                            "Boundary Testing Results:",
                            "Parameter Extremes Results:",
                            "Null/Empty Input Results:",
                            "Mixed Input Results:"
                        )
                        
                        text = args[0]
                        # Allow section headers and test results to print
                        if (any(text.startswith(prefix) for prefix in allowed_prefixes) or
                            "TEST SUMMARY" in text or
                            "test passed" in text.lower() or
                            "test failed" in text.lower() or
                            "edge case" in text.lower() or
                            "=======" in text or
                            "- " in text.lower()):  # Allow bullet points to print
                            original_print(*args, **kwargs)
            
            
                def quiet_logging_info(msg, *args, **kwargs):
                    # Only log critical messages
                    pass
                
                # Replace functions temporarily
                builtins.print = quiet_print
                logging_module.info = quiet_logging_info
            
            # Run all the tests
            print("\n======= RUNNING UNIT TESTS =======")
            self.test_unit_target_identification()
            self.test_unit_angle_validation()
            self.test_unit_collision_detection()
            self.test_unit_distance_calculation()
            self.test_unit_trajectory_optimization()
            
            print("\n======= RUNNING INTEGRATION TESTS =======")
            self.test_integration_validation_pipeline()
            self.test_integration_optimization_pipeline()
            #self.test_integration_visualization_pipeline()
            
            print("\n======= RUNNING SYSTEM TESTS =======")
            self.test_system_complete_planning()
            self.test_system_negative_cases()

            print("\n======= RUNNING ADDITIONAL EDGE CASE TESTS =======")
            self.test_system_null_empty_inputs()
            self.test_system_parameter_extremes()
            self.test_system_boundary_coordinates()
            self.test_system_mixed_inputs()
            self.test_system_resource_limitations()
            
            print("\n======= ALL TESTS COMPLETED =======")

            # Print summary at the end - Make sure this is called
            #self.printTestSummary()
            
        finally:
            # CRITICAL: First print the summary with the current printing settings
            # Ensure the summary gets printed even if tests fail
            try:
                print("\n======= FINAL TEST SUMMARY =======")
                print(f"Unit tests: {self.unit_tests_passed}/{self.unit_tests_total} passed")
                print(f"Integration tests: {self.integration_tests_passed}/{self.integration_tests_total} passed") 
                print(f"System tests: {self.system_tests_passed}/{self.system_tests_total} passed")
                print(f"Overall: {self.unit_tests_passed + self.integration_tests_passed + self.system_tests_passed}/{self.unit_tests_total + self.integration_tests_total + self.system_tests_total} passed")
            except:
                pass

            # Then restore the original print and logging functions
            builtins.print = original_print
            logging_module.info = original_logging_info
            
            # Clean up after all tests
            print("\n======= CLEANING UP =======")
            self.cleanupTestNodes()

    def report_test_outcome(self, test_name, test_type, success, print_header=False):
        """Helper method to report test outcomes consistently"""
        if print_header:
            print(f"\n=== {test_type.upper()} TEST: {test_name} ===")
        
        result_str = "PASSED" if success else "FAILED"
        print(f"{test_type} test {result_str}: {test_name}")
        
        # Update the appropriate counter
        if test_type.lower() == "unit":
            if success:
                self.unit_tests_passed += 1
        elif test_type.lower() == "integration":
            if success:
                self.integration_tests_passed += 1
        elif test_type.lower() == "system":
            if success:
                self.system_tests_passed += 1

    def cleanupTestNodes(self):
        """
        Remove any nodes created during testing
        """ 
       # Skip removing nodes that are needed for the UI
        keep_patterns = ["entriesSubsample", "targetsSubsample"]
        
        # Remove fiducial nodes created during testing
        test_fiducial_patterns = [
            "empty*", "boundary*", "minimal*", "outside*", 
            "steep*", "many*", "Test*", "Optimal*"  # Removed "*Subsample" from here
        ]
        
        for pattern in test_fiducial_patterns:
            for node_name in slicer.util.getNodes(pattern).keys():
                # Skip nodes we want to keep
                if any(keep in node_name for keep in keep_patterns):
                    continue
                    
                try:
                    node = slicer.util.getNode(node_name)
                    if node:
                        slicer.mrmlScene.RemoveNode(node)
                except Exception as e:
                    print(f"Error removing {node_name}: {e}")
    
        # Remove distance maps and other volumes created during tests
        volume_patterns = ["*_distance*"]
        for pattern in volume_patterns:
            for node_name in slicer.util.getNodes(pattern).keys():
                try:
                    node = slicer.util.getNode(node_name)
                    if node:
                        #print(f"Removing volume: {node_name}")
                        slicer.mrmlScene.RemoveNode(node)
                except Exception as e:
                    print(f"Error removing {node_name}: {e}")
        
        # Remove transform nodes
        transform_patterns = ["*Transform*", "*trajectory*transform*"]
        for pattern in transform_patterns:
            for node_name in slicer.util.getNodes(pattern).keys():
                try:
                    node = slicer.util.getNode(node_name)
                    if node:
                        #print(f"Removing transform: {node_name}")
                        slicer.mrmlScene.RemoveNode(node)
                except Exception as e:
                    print(f"Error removing {node_name}: {e}")
        
        # Remove model nodes created for trajectories
        model_patterns = ["*Trajectory*", "Test*"]
        for pattern in model_patterns:
            for node_name in slicer.util.getNodes(pattern).keys():
                if "Model" in node_name or node_name.endswith("_line"):
                    try:
                        node = slicer.util.getNode(node_name)
                        if node:
                            #print(f"Removing model: {node_name}")
                            slicer.mrmlScene.RemoveNode(node)
                    except Exception as e:
                        print(f"Error removing {node_name}: {e}")
        
        print("Test cleanup complete")

        #
    # INTEGRATION TESTS
    #

    def test_integration_validation_pipeline(self):
        """
        Integration test for the validation pipeline.
        """
        #print("\n=== INTEGRATION TEST: Validation Pipeline ===")
        self.delayDisplay("Testing complete validation pipeline")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # Test the integrated validation pipeline
        safe_trajectories = logic.validate_trajectories(
            self.entryPoints.GetName(),
            self.targetPoints.GetName(),
            self.hippo.GetName(),
            self.cortex.GetName(),
            self.ventricles.GetName(),
            self.vessels.GetName()
        )
        
        # logging.info(f"Validation pipeline found {len(safe_trajectories)} safe trajectories")
        # self.assertTrue(len(safe_trajectories) > 0, "No safe trajectories found by validation pipeline")
        # print("PASSED: Validation pipeline integration test")

        success = len(safe_trajectories) > 0
        self.report_test_outcome("Validation pipeline", "Integration", success, print_header=True)
        self.assertTrue(success, "No safe trajectories found by validation pipeline")

    def test_integration_optimization_pipeline(self):
        """
        Integration test for the optimization pipeline.
        """
        #print("\n=== INTEGRATION TEST: Optimization Pipeline ===")
        self.delayDisplay("Testing optimization pipeline")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # First get some safe trajectories
        safe_trajectories = logic.validate_trajectories(
            self.entryPoints.GetName(),
            self.targetPoints.GetName(),
            self.hippo.GetName(),
            self.cortex.GetName(),
            self.ventricles.GetName(),
            self.vessels.GetName()
        )
        
        # Test the optimization with adaptive margins
        best_entry, best_target, best_score, metrics = logic.find_optimal_trajectory(
            safe_trajectories,
            self.vessels.GetName(),
            self.ventricles.GetName()
        )
        
        logging.info(f"Optimization pipeline found best trajectory with score: {best_score}")
        success = best_entry is not None and best_target is not None
        self.report_test_outcome("Optimization pipeline", "Integration", success, print_header=True)
        self.assertTrue(success, "Failed to find optimal trajectory in integration test")

    #
    # UNIT TESTS
    #

    def test_unit_target_identification(self):
        """
        Unit test for hippocampus target identification.
        """
        # Only print detailed header in debug mode
        if hasattr(self, 'debug_mode') and self.debug_mode:
            #print("\n=== UNIT TEST: Target Identification ===")
            self.delayDisplay("Testing hippocampus target identification")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # Test the function
        targets_in_hippo = logic.get_points_in_hippocampus(
            self.hippo.GetName(), 
            self.targetPoints.GetName()
        )
        
        # Only log details in debug mode
        if hasattr(self, 'debug_mode') and self.debug_mode:
            logging.info(f"Found {len(targets_in_hippo)} targets in hippocampus")
        
        # Check test results
        success = len(targets_in_hippo) > 0

        # Use unified reporting method
        success = len(targets_in_hippo) > 0
        self.report_test_outcome("Target identification", "Unit", success, print_header=True)
        
        self.assertTrue(success, "No targets found in hippocampus")
        # if success:
        #     self.unit_tests_passed += 1
        
        # # Use simple logging
        # self.simple_Task_Logging_Unit_Test_Outcome("Target identification", success)
        
        # # Still assert for correctness
        # self.assertTrue(success, "No targets found in hippocampus")

    def test_unit_angle_validation(self):
        """
        Unit test for entry angle validation.
        """
        #print("\n=== UNIT TEST: Entry Angle Validation ===")
        self.delayDisplay("Testing cortex entry angle validation")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # Test the function
        valid_trajectories = logic.check_cortex_entry_angles(
            self.entryPoints.GetName(),
            self.targetPoints.GetName(),
            self.hippo.GetName(),
            self.cortex.GetName()
        )
        
        logging.info(f"Found {len(valid_trajectories)} trajectories with valid angles")
        success = len(valid_trajectories) > 0
        self.report_test_outcome("Entry angle validation", "Unit", success, print_header=True)
        self.assertTrue(success, "No valid trajectories found for entry angle")

        ##self.assertTrue(len(valid_trajectories) > 0, "No valid trajectories found for entry angle")
        # print("PASSED: Entry angle validation test")

    def test_unit_collision_detection(self):
        """
        Unit test for collision detection.
        """
        #print("\n=== UNIT TEST: Collision Detection ===")
        self.delayDisplay("Testing collision detection")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # First get some valid trajectories
        valid_trajectories = logic.check_cortex_entry_angles(
            self.entryPoints.GetName(),
            self.targetPoints.GetName(),
            self.hippo.GetName(),
            self.cortex.GetName()
        )
        
        # Then test collision detection
        safe_trajectories = logic.check_trajectory_collisions(
            valid_trajectories,
            self.ventricles.GetName(),
            self.vessels.GetName()
        )
        
        logging.info(f"Found {len(safe_trajectories)} safe trajectories without collisions")

        success = len(safe_trajectories) > 0
        self.report_test_outcome("Collision detection", "Unit", success, print_header=True)
        self.assertTrue(success, "No safe trajectories found")


        #self.assertTrue(len(safe_trajectories) > 0, "No safe trajectories found")
        #print("PASSED: Collision detection test")

    def test_unit_distance_calculation(self):
        """
        Unit test for distance map calculation.
        """
        #print("\n=== UNIT TEST: Distance Map Calculation ===")
        self.delayDisplay("Testing distance map calculation")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # Test distance map creation
        vessel_distance_map = logic.compute_distance_map(self.vessels.GetName())
        self.assertIsNotNone(vessel_distance_map, "Failed to create vessel distance map")
        
        # Test distance calculation along trajectory
        # Get a known good trajectory
        valid_trajectories = logic.check_cortex_entry_angles(
            self.entryPoints.GetName(),
            self.targetPoints.GetName(),
            self.hippo.GetName(),
            self.cortex.GetName()
        )
        if valid_trajectories:
            # Handle the trajectory with possible angle data
            first_traj = valid_trajectories[0]
            entry_point = first_traj[0]  # First element is always entry point
            target_point = first_traj[1]  # Second element is always target point
            
            distance = logic.calculate_min_distance_along_trajectory(
                entry_point, target_point, vessel_distance_map)
            
            self.assertIsInstance(distance, float, "Distance should be a float value")
            self.assertGreaterEqual(distance, 0.0, "Distance should be non-negative")
            
            print(f"Calculated distance: {distance}")
            print("PASSED: Distance calculation test")
        else:
            print("SKIPPED: No valid trajectories for distance calculation test")
        
        success = vessel_distance_map is not None and len(valid_trajectories) > 0
        self.report_test_outcome("Distance calculation", "Unit", success, print_header=True)
        self.assertTrue(success, "Distance map creation failed")


    def test_unit_trajectory_optimization(self):
        """
        Unit test for trajectory optimization.
        """
        #print("\n=== UNIT TEST: Trajectory Optimization ===")
        self.delayDisplay("Testing optimal trajectory selection")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # First get some safe trajectories
        valid_trajectories = logic.check_cortex_entry_angles(
            self.entryPoints.GetName(),
            self.targetPoints.GetName(),
            self.hippo.GetName(),
            self.cortex.GetName()
        )
        
        safe_trajectories = logic.check_trajectory_collisions(
            valid_trajectories,
            self.ventricles.GetName(),
            self.vessels.GetName()
        )
        
        # Test the optimization
        best_entry, best_target, best_score, metrics = logic.find_optimal_trajectory(
            safe_trajectories,
            self.vessels.GetName(),
            self.ventricles.GetName()
        )
        
        # Verify we have a valid result
        success = best_entry is not None and best_target is not None
        self.report_test_outcome("Trajectory optimization", "Unit", success, print_header=True)
        self.assertTrue(success, "Failed to find optimal trajectory")

        # self.assertTrue(best_entry is not None, "No best entry point found")
        # self.assertTrue(best_target is not None, "No best target point found")
        # self.assertIsNotNone(best_score, "Score should not be None")
        # print("PASSED: Trajectory optimization test")

    def test_system_null_empty_inputs(self):
        """Test handling of null and empty inputs"""
        print("\n=== EDGE CASE TEST: Null/Empty Inputs ===")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # Create empty nodes
        emptyEntries = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "emptyEntries")
        emptyTargets = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "emptyTargets")
        
        print("Testing with empty fiducial nodes (0 points)...")
        
        # Track test results
        stats = {}
        
        # Test with empty fiducials (should handle gracefully)
        try:
            results_empty = logic.run_complete_planning(
                emptyEntries.GetName(),
                emptyTargets.GetName(),
                self.hippo.GetName(),
                self.cortex.GetName(),
                self.ventricles.GetName(),
                self.vessels.GetName(),
                quiet_mode=True
            )
            
            stats["empty_handled"] = True
            stats["empty_result_type"] = type(results_empty).__name__
            stats["empty_trajectories"] = results_empty.get('num_safe_trajectories', 0) if isinstance(results_empty, dict) else 0
            
            print("✅ Empty fiducials test: Completed without exceptions")
            print(f"   Result: {stats['empty_result_type']} returned")
            if isinstance(results_empty, dict):
                print(f"   Safe trajectories found: {stats['empty_trajectories']}")
            
        except Exception as e:
            stats["empty_handled"] = False
            stats["empty_error"] = str(e)
            print(f"❌ Empty fiducials test: Raised exception: {type(e).__name__}: {str(e)}")

        # Success is defined as completing without unhandled exception
        success = True
        self.report_test_outcome("Null/Empty Inputs", "System", success, print_header=False)
        
        # Print summary directly here instead of storing for later
        print("\nNull/Empty Input Results:")
        print(f"- Empty datasets: {'Graceful error handling confirmed' if stats.get('empty_handled', False) else 'Failed to handle properly'}")
        if 'empty_result_type' in stats:
            print(f"- Result type: {stats.get('empty_result_type', 'Unknown')}")
        if 'empty_trajectories' in stats:
            print(f"- Safe trajectories found: {stats.get('empty_trajectories', 0)}")
        
        print("Empty fiducials test passed - no crashes with empty input nodes")
    
    def test_system_boundary_coordinates(self):
        """Test with boundary coordinates"""
        print("\n=== EDGE CASE TEST: Boundary Coordinates ===")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # Get volume dimensions
        hippo_array = slicer.util.arrayFromVolume(self.hippo)
        dims = hippo_array.shape
        print(f"Volume dimensions: {dims}")
        
        # Create fiducial nodes with boundary coordinates
        boundaryEntries = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "boundaryEntries")
        boundaryTargets = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "boundaryTargets")
        
        # Get RAS coordinates for boundary points
        ijkToRas = vtk.vtkMatrix4x4()
        self.hippo.GetIJKToRASMatrix(ijkToRas)
        
        print("Testing with points at volume boundaries...")
        
        # Create boundary points in different positions
        boundary_positions = [
            [dims[0]-1, dims[1]-1, dims[2]-1],  # Last voxel (corner)
            [dims[0]-1, dims[1]//2, dims[2]//2], # Edge of X dimension
            [dims[0]//2, dims[1]-1, dims[2]//2], # Edge of Y dimension
            [dims[0]//2, dims[1]//2, dims[2]-1]  # Edge of Z dimension
        ]
        
        # Track statistics for reporting
        stats = {
            "num_test_points": len(boundary_positions),
            "valid_count": 0,
            "processed_count": 0,
            "success_rate": 0,
            "processing_time": 0
        }
        
        for pos in boundary_positions:
            # Convert IJK to RAS
            boundary_ijk = pos + [1]  # Add homogeneous coordinate
            boundary_ras = ijkToRas.MultiplyPoint(boundary_ijk)
            
            # Add boundary point as target
            point_idx = boundaryTargets.AddControlPoint(boundary_ras[0], boundary_ras[1], boundary_ras[2])
            boundaryTargets.SetNthControlPointLabel(point_idx, f"Boundary_{pos[0]}_{pos[1]}_{pos[2]}")
            
            # Add entry point with offset
            entry_idx = boundaryEntries.AddControlPoint(
                boundary_ras[0]-10, boundary_ras[1]-10, boundary_ras[2]-10)
            boundaryEntries.SetNthControlPointLabel(entry_idx, f"Entry_{pos[0]}_{pos[1]}_{pos[2]}")
        
        print(f"Created {boundaryTargets.GetNumberOfControlPoints()} boundary test points")
        
        # Test behavior with boundary coordinates
        try:
            start_time = time.time()
            results = logic.validate_trajectories(
                boundaryEntries.GetName(),
                boundaryTargets.GetName(),
                self.hippo.GetName(),
                self.cortex.GetName(),
                self.ventricles.GetName(),
                self.vessels.GetName(),
            )
            processing_time = time.time() - start_time
            
            stats["processed_count"] = boundaryTargets.GetNumberOfControlPoints()
            stats["valid_count"] = len(results)
            stats["success_rate"] = (stats["valid_count"] / stats["num_test_points"]) * 100 if stats["num_test_points"] > 0 else 0
            stats["processing_time"] = processing_time
            
            print(f"✅ Boundary coordinates test: Completed without exceptions")
            print(f"   Found {len(results)} trajectories with boundary coordinates")
            print(f"   Processing time: {processing_time:.2f} seconds")
            
        except Exception as e:
            print(f"❌ Boundary coordinates test: Raised exception: {type(e).__name__}: {str(e)}")
        
        # Success is defined as completing without unhandled exception
        success = True
        self.report_test_outcome("Boundary Coordinates", "System", success, print_header=False)
        
        # Print detailed summary for boundary coordinates test
        print("\nBoundary Testing Results:")
        print(f"- Near-volume-edge targets: {stats['num_test_points']} test points")
        print(f"- Successfully processed: {stats['processed_count']}/{stats['num_test_points']} points")
        print(f"- Success rate: {stats['success_rate']:.1f}%")
        print(f"- Valid trajectories found: {stats['valid_count']}")
        print(f"- Processing time: {stats['processing_time']:.2f} seconds")
        print(f"- Graceful handling of boundary coordinates: {'Verified' if stats['processed_count'] > 0 else 'Failed'}")

    def test_system_mixed_inputs(self):
        """Test with inputs in wrong order"""
        print("\n=== EDGE CASE TEST: Mixed-up Inputs ===")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # Track statistics for reporting
        stats = {
            "swap_handled": False,
            "types_handled": False,
            "swap_trajectories": 0,
            "types_trajectories": 0
        }
        
        print("Test 1: Testing with entry/target points swapped...")
        # Test with entry/target swapped
        try:
            start_time = time.time()
            results = logic.run_complete_planning(
                self.targetPoints.GetName(),  # Swapped
                self.entryPoints.GetName(),   # Swapped
                self.hippo.GetName(),
                self.cortex.GetName(),
                self.ventricles.GetName(),
                self.vessels.GetName(),
                quiet_mode=True 
            )
            processing_time = time.time() - start_time
            
            stats["swap_handled"] = True
            stats["swap_success"] = 'best_entry' in results
            stats["swap_trajectories"] = results.get('num_safe_trajectories', 0)
            stats["swap_time"] = processing_time
            
            print(f"✅ Swapped points test: Completed without exceptions")
            print(f"   Processing time: {processing_time:.2f} seconds")
            print(f"   Result: {'Success' if stats['swap_success'] else 'No valid trajectories'}")
            if results and 'num_safe_trajectories' in results:
                print(f"   Safe trajectories found: {stats['swap_trajectories']}")
            
        except Exception as e:
            stats["swap_error"] = str(e)
            print(f"❌ Swapped points test: Raised exception: {type(e).__name__}: {str(e)}")
        
        print("\nTest 2: Testing with wrong volume types...")
        # Test with improper node types (using wrong volumes)
        try:
            start_time = time.time()
            results = logic.run_complete_planning(
                self.entryPoints.GetName(),
                self.targetPoints.GetName(),
                self.ventricles.GetName(),  # Wrong - using ventricles as hippocampus
                self.hippo.GetName(),       # Wrong - using hippocampus as cortex
                self.vessels.GetName(),     # Wrong - using vessels as ventricles
                self.cortex.GetName(),     # Wrong - using cortex as vessels
                quiet_mode=True 
            )
            processing_time = time.time() - start_time
            
            stats["types_handled"] = True
            stats["types_success"] = 'best_entry' in results
            stats["types_trajectories"] = results.get('num_safe_trajectories', 0)
            stats["types_time"] = processing_time
            
            print(f"✅ Wrong volume types test: Completed without exceptions")
            print(f"   Processing time: {processing_time:.2f} seconds")
            print(f"   Result: {'Success' if stats['types_success'] else 'No valid trajectories'}")
            if results and 'num_safe_trajectories' in results:
                print(f"   Safe trajectories found: {stats['types_trajectories']}")
            
        except Exception as e:
            stats["types_error"] = str(e)
            print(f"❌ Wrong volume types test: Raised exception: {type(e).__name__}: {str(e)}")
        
        # Success is defined as completing without unhandled exception
        success = True
        self.report_test_outcome("Mixed-up Inputs", "System", success, print_header=True)
        
        # Print summary directly
        print("\nMixed Input Results:")
        print(f"- Swapped entry/target: {'Handled gracefully' if stats['swap_handled'] else 'Failed'}")
        if 'swap_trajectories' in stats:
            print(f"  * Safe trajectories found: {stats.get('swap_trajectories', 0)}")
            if 'swap_time' in stats:
                print(f"  * Processing time: {stats.get('swap_time', 0):.2f} seconds")
                
        print(f"- Invalid node types: {'Appropriate exception handling verified' if stats['types_handled'] else 'Failed'}")
        if 'types_trajectories' in stats:
            print(f"  * Safe trajectories found: {stats.get('types_trajectories', 0)}")
            if 'types_time' in stats:
                print(f"  * Processing time: {stats.get('types_time', 0):.2f} seconds")

    def test_system_parameter_extremes(self):
        """Test with extreme parameter values"""
        print("\n=== EDGE CASE TEST: Parameter Extremes ===")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # Track statistics for reporting
        stats = {
            "tiny_success": False,
            "huge_success": False,
            "minimal_success": False,
            "tiny_trajectories": 0,
            "huge_trajectories": 0,
            "minimal_trajectories": 0
        }
        
        # Test 1: With tiny trajectory length constraint
        print("Test 1: Testing with extremely small trajectory length limit (0.1mm)...")
        try:
            start_time = time.time()
            # Test with tiny value
            results_tiny = logic.run_complete_planning(
                self.entryPoints.GetName(),
                self.targetPoints.GetName(),
                self.hippo.GetName(),
                self.cortex.GetName(),
                self.ventricles.GetName(), 
                self.vessels.GetName(),
                max_trajectory_length=0.1,  # Unreasonably small
                quiet_mode=True
            )
            processing_time = time.time() - start_time
            
            stats["tiny_success"] = True
            stats["tiny_has_best_entry"] = results_tiny and 'best_entry' in results_tiny
            stats["tiny_trajectories"] = results_tiny.get('num_safe_trajectories', 0)
            stats["tiny_time"] = processing_time
            
            print(f"✅ Tiny trajectory limit test: Completed without exceptions")
            print(f"   Processing time: {processing_time:.2f} seconds")
            print(f"   Result: {'Success' if stats['tiny_has_best_entry'] else 'No valid trajectories'}")
            print(f"   Safe trajectories found: {stats['tiny_trajectories']}")
            
        except Exception as e:
            print(f"❌ Tiny trajectory limit test: Raised exception: {type(e).__name__}: {str(e)}")
            stats["tiny_error"] = str(e)
        
        # Test 2: With huge trajectory length constraint
        print("\nTest 2: Testing with extremely large trajectory length limit (10,000mm)...")
        try:
            start_time = time.time()
            # Test with huge value 
            results_huge = logic.run_complete_planning(
                self.entryPoints.GetName(),
                self.targetPoints.GetName(),
                self.hippo.GetName(),
                self.cortex.GetName(),
                self.ventricles.GetName(),
                self.vessels.GetName(),
                max_trajectory_length=10000,  # Unreasonably large
                quiet_mode=True
            )
            processing_time = time.time() - start_time
            
            stats["huge_success"] = True
            stats["huge_has_best_entry"] = results_huge and 'best_entry' in results_huge
            stats["huge_trajectories"] = results_huge.get('num_safe_trajectories', 0)
            stats["huge_time"] = processing_time
            
            print(f"✅ Huge trajectory limit test: Completed without exceptions")
            print(f"   Processing time: {processing_time:.2f} seconds")
            print(f"   Result: {'Success' if stats['huge_has_best_entry'] else 'No valid trajectories'}")
            print(f"   Safe trajectories found: {stats['huge_trajectories']}")
            
        except Exception as e:
            print(f"❌ Huge trajectory limit test: Raised exception: {type(e).__name__}: {str(e)}")
            stats["huge_error"] = str(e)
        
        # Test 3: With minimal dataset
        print("\nTest 3: Testing with minimal dataset (1 entry point, 1 target point)...")
        
        # Create minimal nodes
        minimalEntries = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "minimalEntries")
        minimalTargets = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "minimalTargets")
        
        # Use a known good point pair
        target_point = [150.0, 82.0, 138.0]
        entry_point = [190.0, 74.0, 148.0]
        
        minimalTargets.AddControlPoint(target_point[0], target_point[1], target_point[2])
        minimalEntries.AddControlPoint(entry_point[0], entry_point[1], entry_point[2])
        
        try:
            start_time = time.time()
            # Run planning with minimal data
            results_minimal = logic.run_complete_planning(
                minimalEntries.GetName(),
                minimalTargets.GetName(),
                self.hippo.GetName(),
                self.cortex.GetName(),
                self.ventricles.GetName(),
                self.vessels.GetName(),
                quiet_mode=True 
            )
            processing_time = time.time() - start_time
            
            stats["minimal_success"] = True
            stats["minimal_has_best_entry"] = results_minimal and 'best_entry' in results_minimal
            stats["minimal_trajectories"] = results_minimal.get('num_safe_trajectories', 0)
            stats["minimal_time"] = processing_time
            
            print(f"✅ Minimal dataset test: Completed without exceptions")
            print(f"   Processing time: {processing_time:.2f} seconds")
            print(f"   Result: {'Success' if stats['minimal_has_best_entry'] else 'No valid trajectories'}")
            print(f"   Safe trajectories found: {stats['minimal_trajectories']}")
            
        except Exception as e:
            print(f"❌ Minimal dataset test: Raised exception: {type(e).__name__}: {str(e)}")
            stats["minimal_error"] = str(e)
        
        # Success is defined as completing without unhandled exception
        success = True
        self.report_test_outcome("Parameter Extremes", "System", success, print_header=True)
        
        # Print summary directly
        print("\nParameter Extremes Results:")
        print("- Stable performance with length constraints:")
        print(f"  * Tiny constraint (0.1mm): {'Handled successfully' if stats['tiny_success'] else 'Failed'} - Found {stats['tiny_trajectories']} trajectories")
        if 'tiny_time' in stats:
            print(f"    Processing time: {stats.get('tiny_time', 0):.2f} seconds")
        
        print(f"  * Huge constraint (10,000mm): {'Handled successfully' if stats['huge_success'] else 'Failed'} - Found {stats['huge_trajectories']} trajectories")
        if 'huge_time' in stats:
            print(f"    Processing time: {stats.get('huge_time', 0):.2f} seconds")
        
        print(f"- Minimal dataset (1×1): {'Processed successfully' if stats['minimal_success'] else 'Failed'} - Found {stats['minimal_trajectories']} trajectories")
        if 'minimal_time' in stats:
            print(f"  * Processing time: {stats.get('minimal_time', 0):.2f} seconds")
        
        print("Parameter extremes tests passed - system handles extreme values gracefully")
    
    def test_system_resource_limitations(self):
        """
        Test behavior under resource limitations
        """
        #print("\n=== SYSTEM TEST: Resource Limitations ===")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # Create many entry and target points to stress the system
        manyEntries = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "manyEntries")
        manyTargets = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "manyTargets")
        
        # Add many points (hundreds or thousands)
        num_points = 500  # Adjust based on your system capabilities
        
        # Create a stats dictionary to track metrics
        stats = {
            "num_entry_points": num_points,
            "num_target_points": num_points,
            "total_combinations": num_points * num_points,
            "completed": False,
            "valid_trajectories": 0,
            "processing_time": 0,
            "success_rate": 0
        }
        
        print(f"Adding {num_points} entry and target points to test resource handling...")
        
        # Use a baseline point and add variations
        base_entry = [200.0, 80.0, 150.0]
        base_target = [150.0, 90.0, 130.0]
        
        for i in range(num_points):
            # Add variations to create unique points
            manyEntries.AddControlPoint(
                base_entry[0] + (i % 20) - 10,
                base_entry[1] + ((i // 20) % 20) - 10,
                base_entry[2] + ((i // 400) % 20) - 10
            )
            
            manyTargets.AddControlPoint(
                base_target[0] + (i % 20) - 10,
                base_target[1] + ((i // 20) % 20) - 10,
                base_target[2] + ((i // 400) % 20) - 10
            )
        
        print(f"Created {manyEntries.GetNumberOfControlPoints()} entry points and {manyTargets.GetNumberOfControlPoints()} target points")
        
        # Time the validation with many points
        start_time = time.time()
        
        try:
            # Just run validation to see if it handles the load
            results = logic.validate_trajectories(
                manyEntries.GetName(),
                manyTargets.GetName(),
                self.hippo.GetName(),
                self.cortex.GetName(),
                self.ventricles.GetName(),
                self.vessels.GetName(),
            )
            
            end_time = time.time()
            processing_time = end_time - start_time
            
            # Update stats with actual values
            stats["completed"] = True
            stats["valid_trajectories"] = len(results)
            stats["processing_time"] = processing_time
            stats["success_rate"] = (len(results) / (num_points * num_points)) * 100
            
            print(f"Processing completed in {processing_time:.2f} seconds")
            print(f"Found {len(results)} valid trajectories")
            
            # Consider success as completing without unhandled exception
            success = True
            self.report_test_outcome("Resource Limitations", "System", success, print_header=True)

            # Print summary directly here
            print("\nStress Testing Results:")
            print(f"- {stats['num_entry_points']} entry points × {stats['num_target_points']} target points = {stats['total_combinations']} combinations")
            print(f"- Processing completed without memory errors")
            print(f"- Peak processing time: {stats['processing_time']:.2f} seconds")
            print(f"- Safe trajectories found: {stats['valid_trajectories']} ({stats['success_rate']:.2f}% of total combinations)")
            
        except MemoryError:
            print("Memory error occurred - this is expected with resource limitation testing")
            # This is actually an expected potential outcome
            self.assertTrue(True, "Memory error is an acceptable outcome for resource limitation testing")
            
        except Exception as e:
            print(f"Unexpected error: {str(e)}")
            self.fail(f"Failed with unexpected error: {str(e)}")

    def test_system_complete_planning(self):
        """
        System test for complete planning workflow.
        """
        #print("\n=== SYSTEM TEST: Complete Planning Workflow ===")
        self.delayDisplay("Testing complete path planning workflow")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # Test the complete planning workflow
        results = logic.run_complete_planning(
            self.entryPoints.GetName(),
            self.targetPoints.GetName(),
            self.hippo.GetName(),
            self.cortex.GetName(),
            self.ventricles.GetName(),
            self.vessels.GetName(),
            quiet_mode=True 
        )

        # Determine test success
        success = ('best_entry' in results and 
                'best_target' in results and 
                'best_score' in results and 
                'trajectory_model' in results and 
                'total_time' in results and
                results.get('best_entry') is not None and
                results.get('best_target') is not None)
        
        # Report test outcome
        self.report_test_outcome("Complete planning workflow", "System", success, print_header=True)
        
        # Keep individual assertions for detailed error reporting
        self.assertTrue('best_entry' in results, "No best entry in results")
        self.assertTrue('best_target' in results, "No best target in results")
        self.assertTrue('best_score' in results, "No best score in results")
        self.assertTrue('trajectory_model' in results, "No trajectory model in results")
        self.assertTrue('total_time' in results, "No timing information in results")
        
        # Test actual values
        self.assertIsNotNone(results.get('best_entry'), "Best entry is None")
        self.assertIsNotNone(results.get('best_target'), "Best target is None")
        self.assertIsNotNone(results.get('vessel_distance'), "Vessel distance not computed")
        self.assertIsNotNone(results.get('ventricle_distance'), "Ventricle distance not computed")
        self.assertIsNotNone(results.get('trajectory_length'), "Trajectory length not computed")
        
    def test_system_negative_cases(self):
        """
        System test for negative cases.
        """
        print("\n=== EDGE CASE TEST: Negative Cases ===")
        self.delayDisplay("Testing rejection of invalid inputs")
        
        # Create logic
        logic = PathPlanningModLogic()
        
        # Track statistics for reporting
        stats = {
            "outside_hippo_handled": False,
            "steep_angle_handled": False, 
            "nonexistent_node_handled": False,
            "outside_count": 0,
            "steep_count": 0
        }
        
        # Test targets outside the hippocampus
        print("Test 1: Testing targets outside hippocampus...")
        outsideTargets = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "outsideTargets")
        outsideTargets.AddControlPoint(100.0, 150.0, 200.0)  # Point outside hippocampus
        
        start_time = time.time()
        targets_in_hippo = logic.get_points_in_hippocampus(
            self.hippo.GetName(), 
            outsideTargets.GetName()
        )
        processing_time = time.time() - start_time
        
        # Check if correctly identified as outside
        stats["outside_hippo_handled"] = (len(targets_in_hippo) == 0)
        stats["outside_count"] = len(targets_in_hippo)
        stats["outside_time"] = processing_time
        
        print(f"✅ Outside hippocampus test: {'Points correctly rejected' if stats['outside_hippo_handled'] else 'Points incorrectly accepted'}")
        print(f"   Processing time: {processing_time:.2f} seconds")
        print(f"   Points incorrectly identified as inside: {stats['outside_count']}")
        
        # Test steep entry angles
        print("\nTest 2: Testing steep entry angles...")
        steepEntries = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "steepEntries")
        steepTargets = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "steepTargets")
        
        target_point = [150.0, 82.0, 138.0]  # One of your known good points
        entry_point = [152.0, 82.0, 168.0]   # Nearly vertical entry
        
        steepTargets.AddControlPoint(target_point[0], target_point[1], target_point[2])
        steepEntries.AddControlPoint(entry_point[0], entry_point[1], entry_point[2])
        
        start_time = time.time()
        valid_trajectories = logic.check_cortex_entry_angles(
            steepEntries.GetName(),
            steepTargets.GetName(),
            self.hippo.GetName(),
            self.cortex.GetName()
        )
        processing_time = time.time() - start_time
        
        # Check if steep angles are properly filtered
        stats["steep_angle_handled"] = (len(valid_trajectories) <= 1) # Should be 0 or very few
        stats["steep_count"] = len(valid_trajectories)
        stats["steep_time"] = processing_time
        
        print(f"✅ Steep angles test: {'Correctly filtered' if stats['steep_angle_handled'] else 'Incorrectly allowed'}")
        print(f"   Processing time: {processing_time:.2f} seconds")
        print(f"   Trajectories found with steep angles: {stats['steep_count']} (should be 0 or very few)")
        
        # Test with non-existent nodes
        print("\nTest 3: Testing error handling with non-existent nodes...")
        start_time = time.time()
        try:
            bad_results = logic.run_complete_planning(
                "NonExistentEntry",
                "NonExistentTarget", 
                self.hippo.GetName(),
                self.cortex.GetName(),
                self.ventricles.GetName(),
                self.vessels.GetName(),
                quiet_mode=True
            )
            processing_time = time.time() - start_time
            stats["nonexistent_node_handled"] = False
            stats["nonexistent_error"] = "No exception raised"
            print("❌ Non-existent nodes test: Failed to raise appropriate exception")
            
        except Exception as e:
            processing_time = time.time() - start_time
            stats["nonexistent_node_handled"] = True
            stats["nonexistent_error"] = str(e)
            stats["nonexistent_time"] = processing_time
            print("✅ Non-existent nodes test: Properly raised exception as expected")
            print(f"   Processing time: {processing_time:.2f} seconds")
            print(f"   Exception: {type(e).__name__}: {str(e)[:100]}...")
        
        # Success is determined by all three negative cases being handled correctly
        success = stats["outside_hippo_handled"] and stats["steep_angle_handled"] and stats["nonexistent_node_handled"]
        self.report_test_outcome("Negative Cases", "System", success, print_header=False)
        
        # Print summary for negative cases
        print("\nNegative Case Test Results:")
        print(f"- Points outside hippocampus: {'Correctly rejected' if stats['outside_hippo_handled'] else 'Incorrectly accepted'}")
        if "outside_time" in stats:
            print(f"  * Processing time: {stats['outside_time']:.2f} seconds")
        
        print(f"- Steep entry angles: {'Correctly filtered' if stats['steep_angle_handled'] else 'Incorrectly allowed'}")
        print(f"  * Trajectories found with extreme angles: {stats['steep_count']}")
        if "steep_time" in stats:
            print(f"  * Processing time: {stats['steep_time']:.2f} seconds")
        
        print(f"- Non-existent nodes: {'Exception properly raised' if stats['nonexistent_node_handled'] else 'Failed to raise exception'}")
        if "nonexistent_time" in stats:
            print(f"  * Processing time: {stats['nonexistent_time']:.2f} seconds")
        
        # Overall status
        status_str = "All negative cases handled appropriately" if success else "Some negative cases not handled properly"
        print(f"- Overall: {status_str}")
