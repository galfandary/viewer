#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

/*######################################################################*/

class win_t {
    Display           *m_dpy;
    Window             m_win;
    XEvent             m_xe;
    GLXContext         m_glx;
    bool               m_dblBuffer;
    gui_viewer_t *m_viewer;

    gui_state_t getState() {
        gui_state_t state;
        if (m_xe.xbutton.state&Mod1Mask) state.alt = true;
        if (m_xe.xbutton.state&ShiftMask) state.shift = true;
        if (m_xe.xbutton.state&ControlMask) state.control = true;
        if (m_xe.xbutton.state&Button1Mask) state.mouse1 = true;
        if (m_xe.xbutton.state&Button2Mask) state.mouse2 = true;
        if (m_xe.xbutton.state&Button3Mask) state.mouse3 = true;
        return state;
    }
    void processXEvent() {
        XNextEvent(m_dpy, &m_xe);
        if (!m_viewer) return;
        switch (m_xe.type) {
        case ButtonPress:
            if (m_xe.xbutton.button == 4 || m_xe.xbutton.button == 5)
                m_viewer->wheel(m_xe.xbutton.button == 4, getState());
            else
                m_viewer->press(m_xe.xbutton.button,
                                m_xe.xmotion.x, m_xe.xmotion.y, getState());
            break;
        case ButtonRelease:
            m_viewer->release(m_xe.xbutton.button);
            break;
        case KeyPress:
            m_viewer->press(XLookupKeysym(&m_xe.xkey, 0), getState());
            break;
        case KeyRelease:
            break;
        case MotionNotify:
            m_viewer->move(m_xe.xmotion.x, m_xe.xmotion.y, getState());
            break;
        case Expose:
            break;
        case ConfigureNotify:
            m_viewer->resize(m_xe.xconfigure.width, m_xe.xconfigure.height);
        }
    }

public:
    virtual int loop() {
        for (;;) {
            do processXEvent(); while (XPending(m_dpy));
            if (m_viewer) m_viewer->repaint();
        }
        return 0;
    }
    virtual int setVisible(bool flag) {
        if (flag) XMapWindow(m_dpy, m_win); else XUnmapWindow(m_dpy, m_win);
        return 0;
    }
    int create(const gui_data_t &data) {
        int errorBase, eventBase;
        if (!(m_dpy = XOpenDisplay(0)) ||
            !glXQueryExtension(m_dpy, &errorBase, &eventBase))
            return -1;

        int doubleBufferVisual[] = {
            GLX_RGBA,           // Needs to support OpenGL
            GLX_DEPTH_SIZE, 16, // Needs to support a 16 bit depth buffer
            GLX_DOUBLEBUFFER,   // Needs to support double-buffering
            None                // end of list
        };

        int singleBufferVisual[] = {
            GLX_RGBA,           // Needs to support OpenGL
            GLX_DEPTH_SIZE, 16, // Needs to support a 16 bit depth buffer
            None                // end of list
        };

        m_dblBuffer = true;
        XVisualInfo *xvi = glXChooseVisual(m_dpy, DefaultScreen(m_dpy), doubleBufferVisual);

        if (!xvi) {
            // If we can't find a double-bufferd visual, try for a single-buffered visual...
            xvi = glXChooseVisual(m_dpy, DefaultScreen(m_dpy), singleBufferVisual);
            if (!xvi) return -1;
            m_dblBuffer = false;
        }

        // Create an OpenGL rendering context
        m_glx = glXCreateContext(m_dpy, xvi, 0, GL_TRUE);
        if (!m_glx) return -1;

        // Create an X colormap since we're probably not using the default visual 
        Colormap colorMap = XCreateColormap(m_dpy, RootWindow(m_dpy, xvi->screen), xvi->visual, AllocNone);

        XSetWindowAttributes windowAttributes;
        windowAttributes.colormap = colorMap;
        windowAttributes.border_pixel = 0;
        windowAttributes.event_mask   =
            ExposureMask           |
            VisibilityChangeMask   |
            KeyPressMask           |
            KeyReleaseMask         |
            ButtonPressMask        |
            ButtonReleaseMask      |
            PointerMotionMask      |
            StructureNotifyMask    |
            SubstructureNotifyMask |
            FocusChangeMask;

        // Create an X window with the selected visual
        m_win = XCreateWindow(m_dpy, RootWindow(m_dpy, xvi->screen),
                              data.xoffset, data.yoffset, data.width, data.height, 0, /* Border width */
                              xvi->depth, InputOutput, xvi->visual,
                              CWBorderPixel|CWColormap|CWEventMask,
                              &windowAttributes);

        XSetStandardProperties(m_dpy, m_win, data.title, data.title, None, 0, 0, 0);

        // Bind the rendering context to the window
        glXMakeCurrent(m_dpy, m_win, m_glx);
        return 0;
    }
    virtual int message(const char *title, const char *msg) const {
        printf("%s: %s\n", title, msg);
        return 0;
    }
    virtual int close() {
        return 0;
    }
    virtual int swap() {
        if (m_dblBuffer) glXSwapBuffers(m_dpy, m_win); else glFlush();
        return 0;
    }
    virtual int repaint() {
        return 0;
    }
    virtual int addViewer(gui_viewer_t *viewer) {
        m_viewer = viewer;
        return 0;
    }
    virtual int destroy() {
        delete this;
        return 0;
    }
    win_t() : m_viewer(0) {
    }
    virtual ~win_t() {
    }
};

/*######################################################################*/

inline int gui_create(win_t *&win, const gui_data_t &data)
{
    win_t *cwin = new win_t(); win = cwin;
    return cwin ? cwin->create(data) : -1;
}
