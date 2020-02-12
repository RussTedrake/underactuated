import asyncio
import sys
from IPython import get_ipython
from IPython.display import display
from ipywidgets.widgets import FloatSlider

from pydrake.systems.framework import VectorSystem

def setup_matplotlib_backend():
    '''
    Helper to support multiple workflows:
        1) nominal -- running locally w/ jupyter notebook
        2) unit tests (no ipython, backend is template)
        3) binder -- does have notebook backend
        4) colab -- does NOT have notebook backend
    Puts the matplotlib backend into notebook mode, if possible,
    otherwise falls back to inline mode.
    Returns True iff the final backend is interactive.
    '''
    ipython = get_ipython()
    if ipython is not None: 
        try:
            ipython.run_line_magic("matplotlib", "notebook")
        except KeyError:
            ipython.run_line_magic("matplotlib", "inline")
        
    # import needs to happen after the backend is set.
    import matplotlib.pyplot as plt
    from matplotlib.rcsetup import interactive_bk
    return plt.get_backend() in interactive_bk    


# Inspired by https://github.com/Kirill888/jupyter-ui-poll but there *must* be a
# better way.  And I could probably make it a lot more efficent (e.g. by not
# doing the entire logic every time, but grabbing events before sim, and
# restoring them after).
def update_widgets(num_ui_events_to_process=1):
    shell = get_ipython()
    # Ok to do nothing if running from console
    if shell is None:
        return
    kernel = shell.kernel
    events = []
    kernel.shell_handlers['execute_request'] = lambda *e: events.append(e)
    current_parent = (kernel._parent_ident, kernel._parent_header)

    for _ in range(num_ui_events_to_process):
        # ensure stdout still happens in the same cell
        kernel.set_parent(*current_parent)
        kernel.do_one_iteration()
        kernel.set_parent(*current_parent)

    kernel.shell_handlers['execute_request'] = kernel.execute_request

    def _replay_events(shell, events):
        kernel = shell.kernel
        sys.stdout.flush()
        sys.stderr.flush()
        for stream, ident, parent in events:
            kernel.set_parent(ident, parent)
            if kernel._aborting:
                kernel._send_abort_reply(stream, parent, ident)
            else:
                kernel.execute_request(stream, ident, parent)

    loop = asyncio.get_event_loop()
    if loop.is_running():
        loop.call_soon(lambda: _replay_events(shell, events))
    else:
        warn(
            'Automatic execution of scheduled cells only works with asyncio based ipython'
        )


# TODO(russt): generalize this to e.g. WidgetSystem (should work for any widget,
# or list of widgets).
class SliderSystem(VectorSystem):

    def __init__(self, min, max, value=0, description=""):
        # 0 inputs, 1 output.
        VectorSystem.__init__(self, 0, 1)
        self.slider = FloatSlider(value=value, description=description, min=min, max=max, continuous_update=True)

        display(self.slider)

    def DoCalcVectorOutput(self, context, unused, unused2, output):
        update_widgets()
        output[:] = self.slider.value
