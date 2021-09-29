#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>
#include <GL/gl.h>

#include "util.hpp"
#include "3ds.hpp"
#include "collide.hpp"

#ifdef _WIN32
# define APIENTRY __stdcall
# define WINGDIAPI __declspec(dllimport)
# define APP_MAIN APIENTRY WinMain(void *dpy, void *dpy0, const char *argv, int show)
#else
# define APP_MAIN main(int argc, char **argv)
# include "xwin.hpp"
#endif

/*######################################################################*/

static char s_log[] = "viewer.log";
inline double square(double x) { return x * x; }
inline double distance(const double *p, const double *q) {
    return sqrt(square(p[0] - q[0]) + square(p[1] - q[1]) + square(p[2] - q[2]));
}
inline double norm2(const double *v) { return square(v[0]) + square(v[1]) + square(v[2]); }
inline void scale3(double s, double *v) { *v++ *= s; *v++ *= s; *v++ *= s; }
inline void add3(double *v, const double *w) { *v++ += *w++; *v++ += *w++; *v++ += *w++; }
inline void sub3(double *v, const double *w) { *v++ -= *w++; *v++ -= *w++; *v++ -= *w++; }
inline void init3(double *v, double x, double y, double z) { *v++ = x; *v++ = y; *v++ = z; }
inline void copy3(double *v, const double *w) { memcpy(v, w, 3 * sizeof(double)); }
inline void copy6(double v[2][3], const double w[2][3]) { memcpy(v, w, 6 * sizeof(double)); }
inline void vflog(const char *file, const char *fmt, va_list arg) {
    FILE *fp = fopen(file, "a"); vfprintf(fp, fmt, arg); fclose(fp);
}
inline void flog(const char *file, const char *fmt, ...) {
    va_list arg; va_start(arg, fmt); vflog(file, fmt, arg); va_end(arg);
}

/*######################################################################*/

class glines_t {
    GLboolean light;
public:
    static void draw(double lines[][3], int size, bool loop = false) {
        if (size) {
            glines_t glines; glBegin(loop ? GL_LINE_LOOP : GL_LINES);
            for (int i = 0; i < size; i++) glVertex3dv(lines[i]);
            glEnd(); glBegin(GL_POINTS);
            for (int j = 0; j < size; j++) glVertex3dv(lines[j]);
        }
    }
    glines_t() {
        light = glIsEnabled(GL_LIGHTING);
        if (light) glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glPointSize(2.0);
        glLineWidth(2.0);
        glColor3ub(255, 0, 0);
    }
    ~glines_t() {
        glEnd();
        glEnable(GL_DEPTH_TEST);
        if (light) glEnable(GL_LIGHTING);
    }
};

/*######################################################################*/

class matrix_t {
    double m_mat[4][4];
public:
    static void initModel() {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }
    static void initProject() {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
    }
    void apply() const {
        glMultMatrixd(&m_mat[0][0]);
    }
    void push() const {
        glPushMatrix();
        apply();
    }
    void getModelview() {
        glGetDoublev(GL_MODELVIEW_MATRIX, &m_mat[0][0]);
    }
    void mult(double *v, double x, double y, double z) const {
        v[0] = m_mat[0][0] * x + m_mat[1][0] * y + m_mat[2][0] * z + m_mat[3][0];
        v[1] = m_mat[0][1] * x + m_mat[1][1] * y + m_mat[2][1] * z + m_mat[3][1];
        v[2] = m_mat[0][2] * x + m_mat[1][2] * y + m_mat[2][2] * z + m_mat[3][2];
    }
    void invert(double *v, double x, double y, double z) const {
        x -= m_mat[3][0]; y -= m_mat[3][1]; z -= m_mat[3][2];
        v[0] = m_mat[0][0] * x + m_mat[0][1] * y + m_mat[0][2] * z;
        v[1] = m_mat[1][0] * x + m_mat[1][1] * y + m_mat[1][2] * z;
        v[2] = m_mat[2][0] * x + m_mat[2][1] * y + m_mat[2][2] * z;
    }
    void translate(double x, double y, double z) {
        m_mat[3][0] += x; m_mat[3][1] += y; m_mat[3][2] += z;
    }
    void translation(double x, double y, double z) {
        m_mat[3][0] = x; m_mat[3][1] = y; m_mat[3][2] = z;
    }
    void rotate(double angle, double x, double y, double z) {
        initModel();
        glRotated(angle, x, y, z);
        apply(); getModelview();
    }
    void scaleLeft(double x, double y, double z) {
        m_mat[0][0] *= x; m_mat[0][1] *= y; m_mat[0][2] *= z;
        m_mat[1][0] *= x; m_mat[1][1] *= y; m_mat[1][2] *= z;
        m_mat[2][0] *= x; m_mat[2][1] *= y; m_mat[2][2] *= z;
    }
    void scaleRight(double x, double y, double z) {
        scale3(x, m_mat[0]); scale3(y, m_mat[1]); scale3(z, m_mat[2]);
        m_mat[3][0] *= x; m_mat[3][1] *= y; m_mat[3][2] *= z;
    }
    void setInverseMatrix(const float m[4][3]) {
        init3(m_mat[0],m[1][1]*m[2][2]-m[2][1]*m[1][2],m[2][1]*m[0][2]-
              m[0][1]*m[2][2],m[0][1]*m[1][2]-m[1][1]*m[0][2]);
        init3(m_mat[1],m[2][0]*m[1][2]-m[1][0]*m[2][2],m[0][0]*m[2][2]-
              m[2][0]*m[0][2],m[1][0]*m[0][2]-m[0][0]*m[1][2]);
        init3(m_mat[2],m[1][0]*m[2][1]-m[2][0]*m[1][1],m[2][0]*m[0][1]-
              m[0][0]*m[2][1],m[0][0]*m[1][1]-m[1][0]*m[0][1]);
		double det = m_mat[0][0]*m[0][0] + m_mat[0][1]*m[1][0] + m_mat[0][2]*m[2][0];
        if (det) scale(1 / det); mult(m_mat[3], -m[3][0], -m[3][1], -m[3][2]);
    }
    void setNodeMatrix(const ds_node_t *node) {
        const double rad2deg = 57.29577951308232087679815481410494;
        rotate(-rad2deg * node->rotate.array[0], node->rotate.array[1],
               node->rotate.array[2], node->rotate.array[3]);
        scaleRight(node->scale.array[0], node->scale.array[1], node->scale.array[2]);
        translate(node->translate.array[0], node->translate.array[1], node->translate.array[2]);
        translate(-node->pivot[0], -node->pivot[1], -node->pivot[2]);
    }
    void reset() {
        memset(m_mat, 0, sizeof(m_mat));
        m_mat[0][0] = m_mat[1][1] = m_mat[2][2] = m_mat[3][3] = 1.0;
    }
    void scale(double s) { scaleRight(s, s, s); }
    void mult(double *v) const { mult(v, v[0], v[1], v[2]); }
    void invert(double *v) const { invert(v, v[0], v[1], v[2]); }
    void getTranslation(double t[3]) const { copy3(t, m_mat[3]); }
    const double *get() const { return &m_mat[0][0]; }
    matrix_t() { reset(); }
};

/*######################################################################*/

class box_t {
    double x0, x1, y0, y1, z0, z1;
public:
    void draw() const {
        if (x0 > x1) return;
        double top[][3] = {{x0,y0,z1},{x1,y0,z1},{x1,y1,z1},{x0,y1,z1}};
        double bottom[][3] = {{x0,y0,z0},{x1,y0,z0},{x1,y1,z0},{x0,y1,z0}};
        double legs[][3] = {{x0,y0,z0},{x0,y0,z1},{x0,y1,z0},{x0,y1,z1},
                            {x1,y0,z0},{x1,y0,z1},{x1,y1,z0},{x1,y1,z1}};
        glines_t::draw(bottom, 4, true);
        glines_t::draw(top, 4, true);
        glines_t::draw(legs, 8);
    }
    void setBox(double cx, double cy, double cz, double rx, double ry, double rz) {
        x0 = cx - rx; x1 = cx + rx;
        y0 = cy - ry; y1 = cy + ry;
        z0 = cz - rz; z1 = cz + rz;
    }
    void addVertex(double x, double y, double z) {
        if (x0 > x1) { x0 = x1 = x; y0 = y1 = y; z0 = z1 = z; return; }
        if (x < x0) x0 = x; else if (x > x1) x1 = x;
        if (y < y0) y0 = y; else if (y > y1) y1 = y;
        if (z < z0) z0 = z; else if (z > z1) z1 = z;
    }
    void addVertex(const float *v) {
        addVertex(v[0], v[1], v[2]);
    }
    void addVertex(double *v, const matrix_t &matrix) {
        matrix.mult(v); addVertex(v[0], v[1], v[2]);
    }
    void inflate(box_t &box, const matrix_t &matrix) const {
        if (x0 > x1) return;
        double v[][3] = {{x0,y0,z0},{x1,y0,z0},{x1,y1,z0},{x0,y1,z0},
                         {x0,y0,z1},{x1,y0,z1},{x1,y1,z1},{x0,y1,z1}};
        for (int i = 0; i < 8; i++) box.addVertex(v[i], matrix);
    }
    void addBox(const box_t &box, const matrix_t &matrix) {
        box.inflate(*this, matrix);
    }
    void getDim(double &radius, double &cx, double &cy, double &cz) const {
        cx = 0.5 * (x0 + x1); cy = 0.5 * (y0 + y1); cz = 0.5 * (z0 + z1);
        double x = cx - x0, y = cy - y0, z = cz - z0;
        radius = sqrt(x*x + y*y + z*z);
    }
    void reset() { x0 = 1; x1 = 0; }
    box_t() { reset(); }
};

/*######################################################################*/

class obb_t {
    box_t    m_box;
    matrix_t m_matrix;
public:
    void drawBox() const {
        m_matrix.push();
        m_box.draw();
        glPopMatrix();
    }
    void getCenter(double &cx, double &cy, double &cz) const {
        double r, v[3];
        m_box.getDim(r, cx, cy, cz);
        m_matrix.mult(v, cx, cy, cz);
        cx = v[0]; cy = v[1]; cz = v[2];
    }
    void scale(double f) {
        double cx0, cy0, cz0, cx1, cy1, cz1;
        getCenter(cx0, cy0, cz0);
        m_matrix.scale(1 + f);
        getCenter(cx1, cy1, cz1);
        m_matrix.translate(cx0 - cx1, cy0 - cy1, cz0 - cz1);
    }
    box_t &getBox() { return m_box; }
    matrix_t &getMatrix() { return m_matrix; }
    const box_t &getBox() const { return m_box; }
    const matrix_t &getMatrix() const { return m_matrix; }
    void inflate(box_t &box) const { m_box.inflate(box, m_matrix); }
};

/*######################################################################*/

class camera_t {
    double m_near;
    double m_radius;
    double m_trans[3];
    double m_center[3];
    matrix_t m_rotate;
    void setModelview() {
        matrix_t::initModel();
        glTranslated(m_trans[0] * m_radius, m_trans[1] * m_radius, m_trans[2] * m_radius);
        m_rotate.apply();
        glTranslated(m_center[0], m_center[1], m_center[2]);
    }
    void setProjection(double ratio) {
        matrix_t::initProject();
        double f = 0.01 * m_radius, A = f, B = f, Z = 1.0 - m_trans[2];
        if (ratio < 1.0) B /= ratio; else A *= ratio;
        glFrustum(-A, A, -B, B, f * m_near, Z * m_radius);
    }
public:
    void apply(double ratio) {
        setProjection(ratio);
        setModelview();
    }
    void setBox(const box_t &box) {
        double cx, cy, cz;
        box.getDim(m_radius, cx, cy, cz);
        m_center[0] = -cx; m_center[1] = -cy; m_center[2] = -cz;
    }
    void translate(double x, double y, double z) {
        m_trans[0] += x; m_trans[1] += y; m_trans[2] += z;
    }
    void rotate(double x, double y) {
        m_rotate.rotate(180 * x, 1, 0, 0);
        m_rotate.rotate(180 * y, 0, 1, 0);
    }
    void world2view(double v[3], double x, double y, double z) const {
        m_rotate.mult(v, x + m_center[0], y + m_center[1], z + m_center[2]);
        for (int i = 0; i < 3; i++) v[i] = v[i] / m_radius + m_trans[i];
    }
    void view2world(double v[3], double x, double y, double z) const {
        double f = -z / m_near; x *= f; y *= f;
        m_rotate.invert(v, x - m_trans[0], y - m_trans[1], z - m_trans[2]);
        for (int i = 0; i < 3; i++) v[i] = v[i] * m_radius - m_center[i];
    }
    void pick(double x, double y, double line[2][3]) const {
        view2world(line[0], x, y, -0.02 * m_near);
        view2world(line[1], x, y, m_trans[2] - 1.0);
    }
    void reset() {
        m_rotate.reset();
        m_trans[0] = m_trans[1] = 0.0; m_trans[2] = -1.0 - m_near;
    }
    camera_t() {
        m_near = 2.0; reset();
    }
};

/*######################################################################*/

class view_t {
    camera_t m_camera;
    double   m_ratio, m_x, m_y;
    int      m_mode, m_width, m_height;
    obb_t   *m_pickObj;
    double   m_pickPointZ;
    double   m_pickPoint[3];
    double   m_translation[3];
public:
    void stop() { m_mode = 0; }
    void reset() { m_camera.reset(); }
    void apply() { m_camera.apply(m_ratio); }
    void setBox(const box_t &box) { m_camera.setBox(box); }
    void pick(double line[2][3]) const { m_camera.pick(m_x, m_y, line); }
    void zoom(double f) { m_camera.translate(0, 0, f); }
    void scale(double f) { if (m_pickObj) m_pickObj->scale(f); }
    void start(int mouse, int x, int y) { m_mode = (m_mode&~3) + mouse; normalize(m_x, m_y, x, y); }
    void setPickPoint(obb_t *obj, const double *point) {
        if (!(m_pickObj = obj)) return;
        m_pickObj->getMatrix().getTranslation(m_translation);
        m_camera.world2view(m_pickPoint, point[0], point[1], point[2]);
        m_pickPointZ = m_pickPoint[2]; copy3(m_pickPoint, point);
        m_mode |= 4;
    }
    void normalize(double &x, double &y, int px, int py) const {
        x = (1.0 + 2.0 * px) / m_width - 1.0;
        y = 1.0 - (1.0 + 2.0 * py) / m_height;
        double m_ratio = (double) m_width / m_height;
        if (m_ratio < 1.0) y /= m_ratio; else x *= m_ratio;
    }
    void resize(int width, int height) {
        glViewport(0, 0, m_width = width, m_height = height);
        m_ratio = (double) m_width / m_height;
    }
    void moveObject(double x, double y) {
        double v[3]; m_camera.view2world(v, x, y, m_pickPointZ);
        for (int i = 0; i < 3; i++) v[i] = m_translation[i] + v[i] - m_pickPoint[i];
        m_pickObj->getMatrix().translation(v[0], v[1], v[2]);
    }
    bool move(int px, int py) {
        if (!(m_mode&3)) return false;
        double x, y; normalize(x, y, px, py);
        if ((m_mode&4) && m_pickObj) { moveObject(x, y); return true; }
        double dx = x - m_x; m_x = x;
        double dy = y - m_y; m_y = y;
        switch(m_mode&3) {
        case 1: m_camera.rotate(-dy, dx); break;
        case 2: m_camera.translate(0, 0, dy); break;
        case 3: m_camera.translate(dx, dy, 0); break;
        }
        return true;
    }
    view_t() : m_mode(0), m_pickObj(0) {}
};

/*######################################################################*/

class lines_t : public callback_t {
    struct line_t {const void *obj; double line[2][3];};
    pointers_t<line_t> m_lines;
    const void *m_obj;
    line_t *newLine() {
        line_t *line = new line_t();
        m_lines.append(line); return line;
    }
    void add(const double p0[3], const double p1[3]) {
        line_t *line = newLine();
        copy3(line->line[0], p0);
        copy3(line->line[1], p1);
        line->obj = m_obj;
    }
public:
    void draw() const {
        int size = 2 * m_lines.size();
        if (size) {
            glines_t glines; glBegin(GL_LINES);
            for (int i = 0; i < size; i++) glVertex3dv(m_lines[i/2]->line[i&1]);
            glEnd(); glBegin(GL_POINTS);
            for (int j = 0; j < size; j++) glVertex3dv(m_lines[j/2]->line[j&1]);
        }
    }
    const void *closest(const double point[3], double pick[3]) const {
        if (!m_lines.size()) return 0;
        int im = 0;
        double m = distance(m_lines[0]->line[0], point);
        for (int i = 1; i < m_lines.size(); i++) {
            double d = distance(m_lines[i]->line[0], point);
            if (d < m) { m = d; im = i; }
        }
        copy3(pick, m_lines[im]->line[0]);
        return m_lines[im]->obj;
    }
    void setObj(const void *obj) { m_obj = obj; }
    void clear() { m_lines.eraseObjects(); }
    void pick(const view_t &view) { view.pick(newLine()->line); }
    virtual bool line(const double l[2][3]) { add(l[0], l[1]); return true; }
    virtual bool point(const double point[3]) { add(point, point); return true; }
    virtual ~lines_t() { clear(); }
};

/*######################################################################*/

struct cmd_t {
    int   argc;
    char *argv[64];
    char  m_buf[4096];
public:
    char *const *find(const char *opt) const {
        for (int i = 0; i < argc; i++)
            if (!strcmp(argv[i], opt)) return argv + ++i;
        return 0;
    }
    const char *file() const {
        const char *opt = argc ? argv[argc - 1] : 0;
        return opt && *opt != '-' ? opt : 0;
    }
    void init(char **args) {
        argc = 0;
        while ((argv[argc] = args[argc])) argc++;
    }
    void init(const char *cmd) {
        argc = 0;
        strcpy(m_buf, cmd);
        char *b = m_buf, *b0 = b;
        while ((b = strchr(b0, ' '))) { *b++ = 0; argv[argc++] = b0; b0 = b; }
        if (*b0) argv[argc++] = b0;
        argv[argc] = 0;
    }
};

/*######################################################################*/

struct image_cb_t {
    virtual void processImage(int width, int height, int bpp, const void *pix) = 0;
    virtual ~image_cb_t() {}
};

inline int imgLoadDummy(const char *file, image_cb_t *cb)
{
    char filePath[256];
    sprintf(filePath, "%s/%s", "images", file);
    FILE *fp = fopen(filePath, "rb");
    int width, height, bpp;
    if (!fp || fscanf(fp, "[%d,%d,%d]", &width, &height, &bpp) != 3) bpp = 0;
    if (!bpp) width = height = 0;
    int size = width * height * bpp;
    void *data = size ? malloc(size) : 0;
    if (fp && data) fread(data, 1, size, fp);
    if (fp) fclose(fp); if (!data) return -1;
    int r = fread(data, 1, size, fp);
    if (r > 0) cb->processImage(width, height, bpp, data);
    free(data); return 0;
}

#ifdef USE_GLU
extern "C" int APIENTRY gluBuild2DMipmaps(GLenum,GLint,GLint,GLint,GLenum,GLenum,const void*);
#else
inline void gluBuild2DMipmaps(int dim, int bpp, int width, int height, int fmt, int rgb, void *pix)
{
    glTexParameteri(dim, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(dim, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexImage2D(dim, 0, bpp, width, height, 0, fmt, rgb, 0);
    glTexSubImage2D(dim, 0, 0, 0, width, height, fmt, rgb, pix);
}
#endif

/*######################################################################*/

class texmap_t {
    struct texture_t : image_cb_t {
        GLuint m_txt;
        float m_x, m_y;
        char m_file[256];
        int rgbMode(int bpp) {
            switch (bpp) {
            case 1: return GL_LUMINANCE;
            case 3: return GL_RGB;
            case 4: return GL_RGBA;
            }
            return GL_RGB;
        }
        virtual void processImage(int width, int height, int bpp, const void *pix) {
            flog("texture.log", "[%04d,%04d,%02d] %s\n", width, height, bpp, m_file);
            int size = width > height ? width : height;
            glBindTexture(GL_TEXTURE_2D, m_txt);
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            gluBuild2DMipmaps(GL_TEXTURE_2D, bpp, width, height, rgbMode(bpp), GL_UNSIGNED_BYTE, pix);
            m_x = (float) width / size; m_y = (float) height / size;
        }
        int load(const char *path, const char *file) {
            if (m_txt) return 0;
            glGenTextures(1, &m_txt);
            if (!m_txt) return -1;
            sprintf(m_file, "%s/%s", path, file);
#           ifdef USE_SDL
            int loadImageSDL(const char *file, image_cb_t *cb);
            if (!loadImageSDL(m_file, this)) return 0;
#           endif
#           ifdef USE_DEVIL
            int loadImageDEVIL(const char *file, image_cb_t *cb);
            if (!loadImageDEVIL(m_file, this)) return 0;
#           endif
            return imgLoadDummy(file, this);
        }
        void vertex(const float *v) const { glTexCoord2f(v[0] * m_x, (1 - v[1]) * m_y); }
        texture_t() : m_txt(0) {}
        ~texture_t() { if (m_txt) glDeleteTextures(1, &m_txt); }
    };
    char m_path[256];
    texture_t *m_texture;
    smap_t<texture_t> m_map;
public:
    void setTexture(const char *file) {
        m_texture = file && *file ? m_map.add(file) : 0;
        if (m_texture && m_texture->load(m_path, file)) m_texture = 0;
        if (m_texture) {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, m_texture->m_txt);
        } else
            glDisable(GL_TEXTURE_2D);
    }
    void setFile(const char *file) {
        strcpy(m_path, file);
        char *sep1 = strrchr(m_path, '/');
        char *sep2 = strrchr(m_path, '\\');
        char *sep = sep1 > sep2 ? sep1 : sep2;
        if (sep) *sep = 0; else strcpy(m_path, ".");
    }
    void vertex(const float *v) const { if (m_texture && v) m_texture->vertex(v); }
    texmap_t() : m_texture(0) { strcpy(m_path, "."); }
};

/*######################################################################*/

class model_t : public obb_t {
protected:
    GLuint            m_list;
    bool              m_transparent;
    char              m_name[64];
    matrix_t          m_top_matrix;
    obj_t            *m_obj;
public:
    void draw(bool box = false) const {
        getMatrix().push();
        glCallList(m_list);
        if (box) getBox().draw();
        glPopMatrix();
    }
    void triangle(const geometry_t *geometry, const texmap_t &texmap) {
        for (int i = 0; i < 3; i++) {
            glNormal3fv(geometry->normal[i]);
            texmap.vertex(geometry->texture[i]);
            glVertex3fv(geometry->vertex[i]);
            getBox().addVertex(geometry->vertex[i]);
        }
        m_obj->append(geometry->vertex);
    }
    void collide(collide_t *collide, const double ray[2][3]) {
        collide->check(m_top_matrix.get(), m_obj, ray, false);
    }
    void setTopMatrix() {
        getMatrix().push();
        m_top_matrix.getModelview();
        glPopMatrix();
    }
    int collide(collide_t *collide, model_t *model, bool first = false) {
        return collide->check(m_top_matrix.get(), m_obj, model->m_top_matrix.get(), model->m_obj, first);
    }
    const char *getName() const { return m_name; }
    bool isTransparent() const { return m_transparent; }
    int getTrg() const { return m_obj ? m_obj->size() : 0; }
    void setTransparent() { m_transparent = true; }
    void build(collide_t *collide) { collide->build(m_obj); }
    void setName(const char *name) { if (name) strcpy(m_name, name); else m_name[0] = 0; }
    model_t(collide_t *collide, const char *name = 0) : m_transparent(false) {
        setName(name);
        collide->createBox(m_obj);
        glNewList(m_list = glGenLists(1), GL_COMPILE);
    }
    virtual ~model_t() { if (m_obj) m_obj->destroy(); }
};

struct models_t : pointers_t<model_t> {
    void drawBox() const { for (int i = 0; i < size(); i++) at(i)->drawBox(); }
    void getBox(box_t &box) const { box.reset(); for (int i = 0; i < size(); i++) at(i)->inflate(box); }
    int getTrg() const { int n = 0; for (int i = 0; i < size(); i++) n += at(i)->getTrg(); return n; }
    void draw(bool box) const {
        for (int i = 0; i < size(); i++) if (!at(i)->isTransparent()) at(i)->draw(box);
        for (int j = 0; j < size(); j++) if (at(j)->isTransparent()) at(j)->draw(box);
    }
    model_t *findModel(const char *name) {
        for (int i = 0; i < size(); i++)
            if (!strcmp(at(i)->getName(), name)) return at(i);
        return 0;
    }
};

/*######################################################################*/

class node_t : public obb_t {
    int m_id;
    model_t *m_model;
    pointers_t<node_t> m_children;
    void drawTransparent(bool isTransparent) const {
        getMatrix().push();
        for (int i = 0; i < m_children.size(); i++)
            m_children[i]->drawTransparent(isTransparent);
        if (m_model && m_model->isTransparent() == isTransparent) m_model->draw();
        glPopMatrix();
    }
    void resetBox() {
        getBox().reset();
        if (m_model) m_model->inflate(getBox());
        for (int i = 0; i < m_children.size(); i++) {
            m_children[i]->resetBox();
            m_children[i]->inflate(getBox());
        }
    }
    void setTopMatrix() {
        getMatrix().push();
        for (int i = 0; i < m_children.size(); i++) m_children[i]->setTopMatrix();
        if (m_model) m_model->setTopMatrix();
        glPopMatrix();
    }
public:
    void collide(collide_t *collide, double ray[2][3], lines_t *lines = 0) const {
        for (int i = 0; i < m_children.size(); i++) {
            if (lines) lines->setObj(m_children[i]);
            m_children[i]->collide(collide, ray);
        }
        if (m_model) m_model->collide(collide, ray);
    }
    void draw(bool box = false) const {
        drawTransparent(false);
        drawTransparent(true);
        if (!box) return;
        for (int i = 0; i < m_children.size(); i++) m_children[i]->drawBox();
    }
    node_t *getNode(int id) {
        node_t *node; if (id == m_id) return this;
        for (int i = 0; i < m_children.size(); i++)
            if ((node = m_children[i]->getNode(id))) return node;
        return 0;
    }
    void eraseObjects() {
        for (int i = 0; i < m_children.size(); i++) delete m_children[i];
    }
    void reset() {
        matrix_t::initModel();
        setTopMatrix();
        resetBox();
    }
    void clear() { m_children.resize(0); }
    void addChild(node_t *node) { m_children.append(node); }
    void removeChild(const node_t *node) { m_children.remove(node); }
    node_t(const ds_node_t *node = 0, node_t *parent = 0, model_t *model = 0) : m_model(model) {
        m_id = node ? node->id : 0;
        if (parent) parent->addChild(this);
        if (node) getMatrix().setNodeMatrix(node);
    }
};

/*######################################################################*/

class torus_t : public model_t {
public:
    torus_t(collide_t *collide, double Ra, double Rb, const char *name = 0) : model_t(collide, name) {
        getBox().setBox(0.0, 0.0, 0.0, Ra + Rb, Ra + Rb, Rb);
        int Na = 600, Nb = 400, N = Na / 6;
        double p2 = 6.283185307179586476925286766559012, Sa = p2 / Na, Sb = p2 / Nb, v0[2][3];
        float color[][4] = {{0.1f, 0.6f, 0.3f, 1.0f}, {0.0f, 0.2f, 0.5f, 0.6f}};
        for (int a = 0; a < Na; a++) {
            double x0 = cos(a * Sa), y0 = sin(a * Sa);
            double x1 = cos((a + 1) * Sa), y1 = sin((a + 1) * Sa);
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color[(a/N)&1]);
            glBegin(GL_TRIANGLE_STRIP);
            for (int b = 0; b <= Nb; b++) {
                double cb = cos(b * Sb), sb = sin(b * Sb), rr = Rb * cb + Ra, zz = Rb * sb;
                double n[2][3] = {{x0 * cb, y0 * cb, sb},{x1 * cb, y1 * cb, sb}};
                double v[2][3] = {{x0 * rr, y0 * rr, zz},{x1 * rr, y1 * rr, zz}};
                glNormal3dv(n[0]); glVertex3dv(v[0]);
                glNormal3dv(n[1]); glVertex3dv(v[1]);
                if (b) { m_obj->append(v0[0],v0[1],v[0]); m_obj->append(v0[1],v[0],v[1]); }
                copy6(v0, v);
            }
            glEnd();
        }
        glEndList();
    }
    virtual ~torus_t() {}
};

/*######################################################################*/

class pyramid_t : public model_t {
public:
    pyramid_t(collide_t *collide) : model_t(collide) {
        getBox().setBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        glBegin(GL_TRIANGLES);
#       define TP glIndexi(1); glColor3d(1.0, 0.0, 0.0); glVertex3i( 0,  1,  0)
#       define FR glIndexi(2); glColor3d(0.0, 1.0, 0.0); glVertex3i( 1, -1,  1)
#       define FL glIndexi(3); glColor3d(0.0, 0.0, 1.0); glVertex3i(-1, -1,  1)
#       define BR glIndexi(3); glColor3d(0.0, 0.0, 1.0); glVertex3i( 1, -1, -1)
#       define BL glIndexi(2); glColor3d(0.0, 1.0, 0.0); glVertex3i(-1, -1, -1)
        TP; FL; FR; TP; FR; BR; TP; BR; BL; TP; BL; FL; FR; FL; BL; BL; BR; FR;
        double v[][3] = {{0.,1.,0.},{1.,-1.,1.},{-1.,-1.,1.},{1.,-1.,-1.},{-1.,-1.,-1.}};
        double *t[][3] = {{v[0],v[1],v[2]},{v[0],v[2],v[3]},{v[0],v[3],v[4]},
                          {v[0],v[4],v[1]},{v[1],v[2],v[3]},{v[2],v[3],v[4]}};
        for (int i = 0; i < 6; i++) m_obj->append(t[i]);
        glEnd();
        glEndList();
    }
    virtual ~pyramid_t() {}
};

/*######################################################################*/

class pick_t {
    node_t  *m_node;
    models_t m_selected;
    double   m_point[3];
    lines_t  m_lines;
public:
    void pick(const node_t &node, collide_t *collide, view_t &view) {
        collide->setCallback(&m_lines);
        m_lines.clear();
        double ray[2][3];
        view.pick(ray);
        m_selected.resize(0);
        node.collide(collide, ray, &m_lines);
        m_node = (node_t *) m_lines.closest(ray[0], m_point);
        view.setPickPoint(m_node, m_point);
    }
    void draw(bool selected, bool lines) {
        if (lines) m_lines.draw();
        if (selected) m_selected.drawBox();
        if (m_node) { m_node->drawBox(); glines_t::draw(&m_point, 1); }
    }
    const node_t *getSelected() const { return m_node; }
    void unSelect() { m_node = 0; }
    pick_t() : m_node(0) {}
};

/*######################################################################*/

class scene_t : private reader_t {
    models_t   m_models;
    int        m_flags;
    collide_t *m_collide;
    texmap_t   m_texmap;
    pick_t     m_pick;
    model_t   *m_3DS;
    int        n_trg;
    lines_t    m_lines;
    node_t     m_tree;

    static void setMaterial(const float *a, const float *d, const float *s, float shininess) {
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, a);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, d);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, s);
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
    }
    static void defaultMaterial() {
        float ambient[] = {0.6f, 0.6f, 0.6f, 1.0f};
        float diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
        float specular[] = {0.0f, 0.0f, 0.0f ,1.0f};
        setMaterial(ambient, diffuse, specular, 0.0f);
    }
    virtual void materialCB(const material_t *material) {
        glEnd(); m_texmap.setTexture(material ? material->texture.file : 0);
        if (material && material->twoside) glDisable(GL_CULL_FACE); else glEnable(GL_CULL_FACE);
        glBegin(GL_TRIANGLES); if (!material) { defaultMaterial(); return; }
        if (material->ambient[3] < 1.0f) m_3DS->setTransparent();
        setMaterial(material->ambient, material->diffuse, material->specular, material->shininess);
    }
    virtual void triangleCB(const geometry_t *geometry) {
        m_3DS->triangle(geometry, m_texmap);
    }
    virtual void objectCB(const char *name) {
        if (name) { m_models.append(m_3DS = new model_t(m_collide, name)); return; }
        glEnd(); glEndList();
        if ((m_flags & F_BUILD) != 0) m_3DS->build(m_collide);
    }
    virtual void matrixCB(const float matrix[4][3]) {
        if (!(m_flags & F_TREE) || !m_3DS || !matrix) return;
        m_3DS->getMatrix().setInverseMatrix(matrix);
    }
    virtual void nodeCB(const ds_node_t *node) {
        if (!(m_flags & F_TREE)) return;
        node_t *parent = m_tree.getNode(node->parent);
        if (parent) new node_t(node, parent, m_models.findModel(node->object));
    }

public:
    enum {
        F_TREE     = 00001,
        F_BBOX     = 00002,
        F_BUILD    = 00004,
        F_LINES    = 00010,
        F_POINTS   = 00020,
        F_SELECTED = 00040
    };
    int collide() {
        int n = 0; clearLines();
        m_collide->setCallback(&m_lines);
        for (int i = 0; i < m_models.size() - 1; i++)
            for (int j = i + 1; j < m_models.size(); j++)
                n += m_models[i]->collide(m_collide, m_models[j]);
        return n;
    }
    void draw() {
        m_tree.draw((m_flags & F_BBOX) != 0);
        m_pick.draw((m_flags & F_SELECTED) != 0, (m_flags & F_POINTS) != 0);
        m_lines.draw();
    }
    bool pick(view_t &view, bool shift) {
        if (shift) m_pick.pick(m_tree, m_collide, view);
        else if (m_flags & F_LINES) m_lines.pick(view);
        return shift || (m_flags & F_LINES);
    }
    void create(const cmd_t &cmd) {
        const char *file = cmd.file();
        if (file) {
            char *const *tree = cmd.find("-tree");
            if (tree) toggle(F_TREE);
            m_texmap.setFile(file);
            read3DS(file, this);
            if (tree) return;
        } else if (cmd.find("-t")) {
            m_models.append(new torus_t(m_collide, 3.0, 1.0));
        } else if (cmd.find("-2")) {
            torus_t *t1 = new torus_t(m_collide, 2.0, 0.99, "Torus1");
            torus_t *t2 = new torus_t(m_collide, 2.0, 0.99, "Torus2");
            t2->getMatrix().rotate(90, 1, 0, 0);
            t2->getMatrix().translate(2, 0, 0);
            m_models.append(t1); m_models.append(t2);
        } else {
            m_models.append(new pyramid_t(m_collide));
            glDisable(GL_LIGHTING);
        }
        for (int i = 0; i < m_models.size(); i++)
            new node_t(0, &m_tree, m_models[i]);
    }
    int getTrg() const { return n_trg; }
    int size() const { return m_models.size(); }
    void reset() { m_tree.reset(); }
    void clearLines() { m_lines.clear(); }
    void setTrg() { n_trg = m_models.getTrg(); }
    void toggle(int flag) { m_flags ^= flag; }
    void deleteSelected() { m_tree.removeChild(m_pick.getSelected()); m_pick.unSelect(); }
    void setTolerance(double clr) { if (m_collide) m_collide->setTolerance(clr); }
    void setClearance(double clr) { if (m_collide) m_collide->setClearance(clr); }
    box_t getBox() const { box_t box; m_tree.inflate(box); return box; }
    scene_t() : m_flags(0), n_trg(0) { collide_t::create(m_collide); }
    ~scene_t() { m_models.eraseObjects(); m_tree.eraseObjects(); if (m_collide) m_collide->destroy(); }
};

/*######################################################################*/

class app_t : public gui_viewer_t {
    view_t      m_view;
    scene_t     m_scene;
    win_t      *m_win;

    void postRepaint() { if (m_win) m_win->repaint(); }
    virtual void resize(int w, int h) { m_view.resize(w, h); }
    virtual void move(int x, int y, const gui_state_t &state) { if (m_view.move(x, y)) postRepaint(); }

    void messageBox(const char *title, const char *fmt, ...) {
        char msg[4096]; va_list arg;
        va_start(arg, fmt); vsprintf(msg, fmt, arg);
        vflog(s_log, fmt, arg); va_end(arg);
        if (m_win) m_win->message(title ? title : "Error", msg);
    }
    void showHelp() {
        messageBox("Usage help",
                   "viewer [-ci] [-sb]\n"
                   "  -sb   single buffered\n"
                   "  -ci   color index\n");
    }
    int collide(int n = 1) {
        int c = 0; double t = clock();
        for (int i = 0; i < n; i++) c = m_scene.collide();
        t = clock() - t;
        messageBox("Collision Detection",
                   "Collision Detection (%d) for scene took: %g milliseconds\n"
                   "Found %d intersections\n", n, t/n, c);
        postRepaint(); return (int) t;
    }
    int test() {
        remove(s_log);
        gui_state_t state;
        press('c', state);
        press('1', state);
        press('2', state);
        return 0;
    }
    void showInfo() {
        messageBox("Information",
                   "Number of objects: %d\n"
                   "Number of triangles: %d\n",
                   m_scene.size(), m_scene.getTrg());
    }
    void resetView() {
        m_scene.reset();
        m_view.reset();
        m_view.setBox(m_scene.getBox());
        postRepaint();
    }
    virtual void wheel(bool up, const gui_state_t &state) {
        if (state.shift)
            m_view.scale(up ? -.2 : .2);
        else
            m_view.zoom(up ? -.2 : .2);
        postRepaint();
    }
    virtual void press(int key, const gui_state_t &state) {
        switch (key) {
        case 'H': showHelp(); break;
        case 'i': showInfo(); break;
        case 'r': resetView(); break;
        case '0': m_scene.setClearance(0); m_scene.setTolerance(1E-6); break;
        case '1': m_scene.setClearance(0.1); collide(); break;
        case '2': m_scene.setClearance(0.5); collide(); break;
        case '3': m_scene.setTolerance(1E-1); collide(); break;
        case '4': m_scene.setTolerance(1E-2); collide(); break;
        case '5': m_scene.setTolerance(1E-3); collide(); break;
        case 'c': collide(); break;
        case 'd': collide(10); break;
        case 'e': collide(100); break;
        case 'f': collide(1000); break;
        case 'l': m_scene.clearLines(); postRepaint(); break;
        case 'b': m_scene.toggle(scene_t::F_BBOX); postRepaint(); break;
        case 'p': m_scene.toggle(scene_t::F_LINES); postRepaint(); break;
        case 's': m_scene.toggle(scene_t::F_SELECTED); postRepaint(); break;
        case 010: m_scene.deleteSelected(); postRepaint(); break;
        case 011: exit(0); break;
        case 033: exit(0); break;
        case 'q': exit(0); break;
        case XK_Tab: exit(0); break;
        case XK_Escape: exit(0); break;
        case XK_Delete: m_scene.deleteSelected(); postRepaint(); break;
        case XK_BackSpace: m_scene.deleteSelected(); postRepaint(); break;
        default: messageBox(0, "key = %x = '%c'\n", key, key);
        }
    }
    virtual void release(int mouse) {
        m_view.stop();
        m_scene.reset();
    }
    virtual void press(int mouse, int x, int y, const gui_state_t &state) {
        m_view.start(mouse, x, y);
        if (m_scene.pick(m_view, state.shift)) postRepaint();
    }
    virtual void repaint() {
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glPushMatrix();
        m_view.apply();
        m_scene.draw();
        glPopMatrix();
        if (m_win) m_win->swap();
    }
    void initGL() {
        glClearColor(0.0, 0.0, 0.0, 1.0); // Black background
        glEnable(GL_BLEND);
        glEnable(GL_DITHER);
        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHTING);
        glEnable(GL_NORMALIZE);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
        glCullFace(GL_BACK);
    }
public:
    void init(void *dpy, const cmd_t &cmd) {
        gui_data_t data = {dpy,"3D Viewer",800,600,100,50,false,false};
        if (cmd.find("-ci")) data.colorIndex = true;
        if (cmd.find("-sb")) data.singleBuffer = true;
        if (!cmd.find("-w") && gui_create(m_win, data)) {
            messageBox(0, "Failed to create a window.\n"); return;
        }
        initGL();
        if (m_win) m_win->addViewer(this);
        if (cmd.find("-h")) showHelp();
        if (cmd.find("-b")) m_scene.toggle(scene_t::F_BUILD);
        m_scene.create(cmd);
        m_scene.setTrg();
        m_view.resize(data.width, data.height);
        resetView();
    }
    void showWindow() { if (m_win) m_win->setVisible(true); }
    int run() { return m_win ? m_win->loop() : test(); }
    app_t() : m_win(0) {}
    ~app_t() { if (m_win) m_win->destroy(); }
};

/*######################################################################*/

int APP_MAIN
{
    app_t app;
    cmd_t cmd;
#   ifndef _WIN32
    void *dpy = 0;
#   endif
    cmd.init(argv);
    app.init(dpy, cmd);
    app.showWindow();
    return app.run();
}
