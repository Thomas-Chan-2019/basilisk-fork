#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


#
# Monte Carlo tests
#
# Purpose:  This script calls a series of bskSim Monte Carlo related simulations to ensure
# that they complete properly.
# Creation Date: Nov 18, 2019
# Recenlty updated: Nov 4, 2024
#


import importlib
import inspect
import os
import shutil
import sys
import pytest

# Check if Bokeh is available
bokeh_spec = importlib.util.find_spec("bokeh")
bokeh_available = bokeh_spec is not None

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples/MonteCarloExamples')

# Skip test if Python version is less than 3.9
@pytest.mark.skipif(sys.version_info < (3, 9),
                    reason="Test has issues with Controller class and older python.")

# Skip test if Bokeh is not available
@pytest.mark.skipif(not bokeh_available,
                    reason="Bokeh is not available. Skipping test.")
@pytest.mark.slowtest
@pytest.mark.scenarioTest

def test_scenarioBskMcScenarios(show_plots):
    # These need to be run in serial such that the data is produced for analysis
    scenarios = ['scenarioBskSimAttFeedbackMC',
                 'scenarioVisualizeMonteCarlo']

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    for i, bskSimCase in enumerate(scenarios):
        # import the bskSim script to be tested
        scene_plt = importlib.import_module(bskSimCase)

        try:
            if i == 0:
                figureList = scene_plt.run(False)
            else:
                scene_plt.run()

        except Exception as err:
            testFailCount += 1
            testMessages.append(f"Error in {bskSimCase}: {str(err)}")

    # Clean up
    if os.path.exists(path + "/../../examples/MonteCarloExamples/scenarioBskSimAttFeedbackMC/"):
        shutil.rmtree(path + "/../../examples/MonteCarloExamples/scenarioBskSimAttFeedbackMC/")

    assert testFailCount < 1, testMessages
