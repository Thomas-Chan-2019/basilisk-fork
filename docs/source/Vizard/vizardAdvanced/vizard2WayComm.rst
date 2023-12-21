
.. _vizard2WayComm:

Using 2-Way Communication with Vizard
=====================================

Enabling 2-Way Communication
----------------------------
Vizard now supports 2-way communication during live-streaming, during which user inputs such
as keystrokes, buttons on VR controllers, or panel selections can be sent back to BSK live to influence
the simulation. This is enabled using the ``liveUserInput`` flag::

    viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, scObject
                                              , liveStream=True
                                              , liveUserInput=True
                                              )

.. caution::
    BSK can only take advantage of 2-way communication during live-stream mode, so ``liveStream`` must be set to **True**.

Creating Panels
---------------
First, let's discuss how to create ``EventDialog`` panels, which contain choices for a user to select. These panels can be configured to display in Vizard at a pre-defined start time, with a prescribed duration and options for a user to select. The following list shows all ``EventDialog`` structure variables.

.. list-table:: ``EventDialog`` variables
    :widths: 20 10 10 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``eventHandlerID``
      - string
      -
      - Yes
      - Name of Vizard event handler to be returned with ``EventReply`` responses
    * - ``displayString``
      - string
      - 
      - Yes
      - Contains the information or choice that should be posed to the user.
    * - ``userOptions``
      - string[]
      - 
      - No
      - Array of display strings, one entry for each user choice that will be shown. If this is empty, the dialog is assumed to be informational only.
    * - ``durationOfDisplay``
      - double
      - nanoseconds
      - No
      - Determines when to close a panel. Default is 0, which leaves the panel on display until closed by user.
    * - ``useSimElapsedTimeForDuration``
      - bool
      -
      - No
      - If true and ``durationOfDisplay`` is set, use the sim elapsed time to calculate when to hide window. If false, use real time (system clock). Default is false.
    * - ``useConfirmationPanel``
      - int
      -
      - No
      - -1 to not show a confirmation panel, 0 to use viz default, 1 to require a user confirmation of their selection.
    * - ``hideOnSelection``
      - int
      -
      - No
      - -1 to continue to show panel , 0 to use viz default, 1 to hide panel after user makes a selection, 2 to destroy panel after user makes a selection
    * - ``dialogFormat``
      - string
      - 
      - No
      - Select format for dialog box: "WARNING", "CAUTION", or none to use viz default format


Here is an example an ``EventDialog`` panel creation::

    powerModePanel = vizInterface.EventDialog()
    powerModePanel.eventHandlerID = "Power Mode Panel"
    powerModePanel.displayString = "Set system power mode:"
    powerModePanel.userOptions.append("Nominal")
    powerModePanel.userOptions.append("Low-Power")
    powerModePanel.useConfirmationPanel = True

    viz.eventDialogs.append(powerModePanel)

.. note::
    The list ``viz.eventDialogs`` sends current panel requests to Vizard as part of the VizMessage, then clears itself before the next timestep. If information in a panel needs to be modified, the same ``EventDialog`` instance (with the same ``eventHandlerID``) can be modified and **re-appended** to ``viz.eventDialogs``. This will cause the panel to re-open if minimized, with updated information. If the panel list needs to be manually cleared, this can be done using ``viz.eventDialogs.clear()``.
	
Handling User Input
-------------------
Responses from panels can be used as inputs back to BSK. The key is that the responses must be read from the :ref:`VizUserInputMsgPayload` message at the desired rate.

From Python, this can be achieved by calling ``scSim.ExecuteSimulation()`` at the desired input reception rate so that responses can be parsed and used to affect the simulation state. 
    
The required structure resembles the following::

    currentTime = 0
    inputTimeStep = macros.sec2nano(5) # Read inputs every 5 seconds
    ...
    scSim.InitializeSimulation()
    for i in range(int(totalDuration/inputTimeStep)):
        currentTime += inputTimeStep
        scSim.ConfigureStopTime(currentTime)
        scSim.ExecuteSimulation()
        
        userInputs = viz.userInputMsg.read()
        keyInputs = userInputs.keyboardInput
        eventInputs = userInputs.eventReplies
        
        # Parse "keyInputs" and "eventInputs", modify sim state

The 2-way communication output message, ``viz.userInputMsg`` , is an instance of :ref:`VizUserInputMsgPayload`. This message fills like a queue: Vizard collects all inputs that were recorded over the last ``scSim.ExecuteSimulation`` call, and hands them all over together. 

.. caution::
    Setting a low input frequency (here, represented by ``inputTimeStep``) could lead to build-up of responses, that may conflict with one another.

This behavior could also be built into a BSK module, in which case the above code structure would not be needed. However, this module would have to hard-code the mappings for different Vizard response types and their associated BSK actions.

Keyboard Parsing
~~~~~~~~~~~~~~~~
There are two types of replies that Vizard can send in return. The ``keyboardInput`` field of the message contains a string of keyboard characters that have been pressed since the last timestep. **Keys will only be recorded if specified.** In the example below, listeners are configured for the keys 'a', 'b', 'c', and 'd'::

    viz.settings.keyboardLiveInput = "abcd"

.. caution::
    Note that Vizard has certain keys pre-programmed as hot-keys for menus and scene actions. If a hot-key is selected as a duplicate listener, Vizard will display a warning, and dual-actions may occur.

To parse ``keyInputs``, search the string for characters of interest::

    if 'a' in keyInputs:
        # 'a' key action
    if 'b' in keyInputs:
        # 'b' key action
    ...

Panel Response Parsing
~~~~~~~~~~~~~~~~~~~~~~
Vizard can also return ``EventReply`` structures, which contain information about selections made within ``EventDialog`` panels. The following list shows all ``EventReply`` structure variables.

.. list-table:: ``EventReply`` variables
    :widths: 20 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Description
    * - ``eventHandlerID``
      - string
      - Name provided when setting up the EventDialog object
    * - ``reply``
      - string
      - Option selected by user
    * - ``eventHandlerDestroyed``
      - bool
      - Was the panel closed and destroyed?


To parse ``eventInputs`` , loop over the list::

    for response in eventInputs:
        eventID = response.eventHandlerID
        eventOption = response.reply
        
        if eventID == "Power Mode Panel":
            if eventOption == "Low-Power":
                # change mode
        elif eventID == ...


See the scenario :ref:`scenarioBasicOrbitStream` for an implemented 2-way communication example.
