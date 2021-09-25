#define rabs  (double)fabs
#define rsqrt (double)sqrt
#define rsin  (double)sin
#define rcos  (double)cos
#define racos (double)acos

const double EPS0((double)1E-6);

/*######################################################################*/

#ifdef DEBUG_COLLIDE
# define dbg(data) fprintf data
#else
# define dbg(data)
#endif

inline double rsq(double r) { return r * r; }

#define RL2(a, b, r) (a r b ? a : b)
#define RL3(a, b, c, r) (a r b ? RL2(a, c, r) : RL2(b, c, r))

#define MN2(f, t) inline t f(t a, t b) { return RL2(a, b, <); }
#define MX2(f, t) inline t f(t a, t b) { return RL2(a, b, >); }
#define MN3(f, t) inline t f(t a, t b, t c) { return RL3(a, b, c, <); }
#define MX3(f, t) inline t f(t a, t b, t c) { return RL3(a, b, c, >); }
#define MN_MX_ALL(f, g, t) MN2(f, t) MX2(g, t) MN3(f, t) MX3(g, t)

MN_MX_ALL(rmin, rmax, double) const int _0 = 0;

/*######################################################################*/

class vec_t {
    double v[3];
public:
    double &operator[](int i) { return v[i]; }
    double operator[](int i) const { return v[i]; }
    void zero() { v[0] = v[1] = v[2] = 0.0; }
    void get(double *w) const { w[0] = v[0]; w[1] = v[1]; w[2] = v[2]; }
    void init(const float *w) { v[0] = w[0]; v[1] = w[1]; v[2] = w[2]; }
    void init(const double *w) { v[0] = (double)w[0]; v[1] = (double)w[1]; v[2] = (double)w[2]; }
    void operator+=(const vec_t &w) { v[0] += w[0]; v[1] += w[1]; v[2] += w[2]; }
    void operator-=(const vec_t &w) { v[0] -= w[0]; v[1] -= w[1]; v[2] -= w[2]; }
    void operator*=(double scale) { v[0] *= scale; v[1] *= scale; v[2] *= scale; }
    vec_t operator+(const vec_t &w) const { return vec_t(v[0] + w[0], v[1] + w[1], v[2] + w[2]); }
    vec_t operator-(const vec_t &w) const { return vec_t(v[0] - w[0], v[1] - w[1], v[2] - w[2]); }
    vec_t operator*(double scale) const { return vec_t(scale * v[0], scale * v[1], scale * v[2]); }
    double dot(const vec_t &w) const { return v[0]*w[0] + v[1]*w[1] + v[2]*w[2]; }
    double absdot(const vec_t &w) const { return rabs(dot(w)); }
    double absnorm() const { return rmax(rabs(v[0]), rabs(v[1]), rabs(v[2])); }
    double norm2() const { return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; }
    double norm() const { return rsqrt(norm2()); }
    void normalize() { operator*=((double)1.0 / norm()); }
    vec_t(double x, double y, double z) { v[0] = x; v[1] = y; v[2] = z; }
    vec_t(const vec_t &v1, const vec_t &v2) {
        v[0] = v1[1] * v2[2] - v1[2] * v2[1];
        v[1] = v1[2] * v2[0] - v1[0] * v2[2];
        v[2] = v1[0] * v2[1] - v1[1] * v2[0];
    }
    vec_t() {}
};

class mat_t {
    vec_t M[3];
public:
    vec_t mult(const vec_t &v) const {
        return vec_t(M[0][0] * v[0] + M[1][0] * v[1] + M[2][0] * v[2],
                     M[0][1] * v[0] + M[1][1] * v[1] + M[2][1] * v[2],
                     M[0][2] * v[0] + M[1][2] * v[1] + M[2][2] * v[2]);
    }
    void mult(const mat_t &M1, const mat_t &M2) {
        M[0] = M2.mult(M1[0]); M[1] = M2.mult(M1[1]); M[2] = M2.mult(M1[2]);
    }
    void multT(const mat_t &M1, const mat_t &M2) {
        M[0] = M2.multT(M1[0]); M[1] = M2.multT(M1[1]); M[2] = M2.multT(M1[2]);
    }
    const vec_t &operator[](int i) const { return M[i]; }
    vec_t &operator[](int i) { return M[i]; }
    vec_t multT(const vec_t &v) const { return vec_t(v.dot(M[0]), v.dot(M[1]), v.dot(M[2])); }
    void operator+=(const vec_t &v) { M[0] += v; M[1] += v; M[2] += v; }
    void operator-=(const vec_t &v) { M[0] -= v; M[1] -= v; M[2] -= v; }
    void operator*=(double scale) { M[0] *= scale; M[1] *= scale; M[2] *= scale; }
    void init(const double *x, const double *y, const double *z) { M[0].init(x); M[1].init(y); M[2].init(z); }
    void identity() { zero(); M[0][0] = M[1][1] = M[2][2] = 1.0; }
    void zero() { M[0].zero(); M[1].zero(); M[2].zero(); }
};

class xform_t : public mat_t {
public:
    vec_t t;
    void reset() { identity(); t.zero(); }
    xform_t() {}
    xform_t(int) { reset(); }
    xform_t(const double *x) { if (x) { init(&x[0], &x[4], &x[8]); t.init(&x[12]); } else reset(); }
};

/*######################################################################*/

#define trg_t mat_t
class trgs_t { public: trg_t t; trgs_t *next; };

class orient_t {
    const double E1;
    double a11, a22, a33, a12, a13, a23; // Central Moments matrix.
    double b11, b22, b33, b12, b13, b23; // Its adjacent matrix.

    // Based on Numerical Recipes in C.
    bool Jacobi3(mat_t &M) {
        int n; double t,c,s,f,g,h;

        for (int i = 0; i < 50; i++) { // Normally takes 4-6 iterations.
            f=rabs(a12); g=rabs(a13); h=rabs(a23); n=0;
            if (g > f) n=1, f=g; if (h > f) n=2, f=h; if (f < E1) return true;

#           define ROT2(a,b) g=a; a=c*a-s*b; b=s*g+c*b
#           define ROTM(p,q) ROT2(M[p][0],M[q][0]); ROT2(M[p][1],M[q][1]); ROT2(M[p][2],M[q][2])
#           define ROT3(aii, ajj, aij) h=(ajj-aii)/2;                                             \
                                       if (f < rabs(h)) g=aij/h, t=g/(1+rsqrt(1+g*g));            \
                                       else g=h/aij, t=(g > 0 ? 1 : -1)/(rabs(g)+rsqrt(1+g*g));   \
                                       c=1/rsqrt(1+t*t); s=t*c; h=t*aij; aii -= h; ajj += h; aij=0

            switch (n) {
            case 0:  ROT3(a11,a22,a12); ROT2(a13,a23); ROTM(0,1); break;
            case 1:  ROT3(a11,a33,a13); ROT2(a12,a23); ROTM(0,2); break;
            default: ROT3(a22,a33,a23); ROT2(a12,a13); ROTM(1,2); break;
            }
        }

        dbg((stderr, "Jacobi3: too many iterations in Jacobi transform.\n"));
        return false;
    }

    bool eigenVec(double l, vec_t &v) {
        double f = rabs(v[0] = b11+l*(a11+l)), g = rabs(v[1] = b22+l*(a22+l)), h = rabs(v[2] = b33+l*(a33+l));
        int n=0; if (g > f) n=1, f=g; if (h > f) n=2, f=h; if (f < E1) return true;
        switch (n) {
        case 0:  v[1] = b12+l*a12; v[2] = b13+l*a13; v.normalize(); return false;
        case 1:  v[0] = b12+l*a12; v[2] = b23+l*a23; v.normalize(); return false;
        default: v[0] = b13+l*a13; v[1] = b23+l*a23; v.normalize(); return false;
        }
    }

    // Based on Graphics Gems IV.
    bool Cromwell(mat_t &M) {
        // Set trace(a) = 0
        double a = (a11 + a22 + a33)/3; a11 -= a; a22 -= a; a33 -= a;

        // Get the adjacent matrix.
        b11 = a22*a33 - a23*a23; b22 = a11*a33 - a13*a13; b33 = a11*a22 - a12*a12;
        b12 = a13*a23 - a12*a33; b13 = a12*a23 - a22*a13; b23 = a12*a13 - a11*a23;

        double b = -(b11 + b22 + b33)/3; if (b < E1) return false;
        double det = a11*b11 + a12*b12 + a13*b13, sq = rsqrt(b), c3 = det/2/b/sq;

        double l1, l2, l3;
        const double sq3 = rsqrt(3.0);

        // Get eigenvalues (naturally sorted).
        if (c3 < -1 + E1 || c3 > 1 - E1) {   // phi = 0, 60; (-sq, -sq, 2sq), (sq, sq, -2sq)
            return false;
        } else if (rabs(c3) < EPS0) {          // phi = 30
            l1 = sq3*sq; l2 = 0; l3 = -l1;
        } else {                             // 0 < phi < 60
            double p = racos(c3)/3, c = rcos(p), s = sq3*rsin(p);
            l1 = 2*sq*c; l2 = -sq*(c - s); l3 = -sq*(c + s);
        }

        // Get eigenvectors.
        if (eigenVec(l1, M[0]) || eigenVec(l2, M[1]) || eigenVec(l3, M[2])) return false;
        return M[0].absdot(M[1]) < E1 && M[0].absdot(M[2]) < E1 && M[1].absdot(M[2]) < E1;
    }

    void setMoments(const vec_t &mean, const trgs_t *t) {
        // Compute Central Moments Matrix.
        a11=a22=a33=a12=a13=a23=0;

        for (; t; t=t->next) for (int i=0; i<3; i++) {
            double m1 = t->t[i][0] - mean[0], m2 = t->t[i][1] - mean[1], m3 = t->t[i][2] - mean[2];
            a11 += m1*m1; a22 += m2*m2; a33 += m3*m3; a12 += m1*m2; a13 += m1*m3; a23 += m2*m3;
        }

        // Set |a[i][j]| <= 1
        double m = rmax(a11, a22, a33);
        a11 /= m; a22 /= m; a33 /= m; a12 /= m; a13 /= m; a23 /= m;
    }

public:
    orient_t(const vec_t &mean, const trgs_t *t, mat_t &M) : E1(10 * EPS0) {
        setMoments(mean, t);
        // Check if already diagonal ...
        if (rabs(a12) < E1 && rabs(a13) < E1 && rabs(a23) < E1) { M.identity(); return; }
        if (Cromwell(M)) return;
        dbg((stderr, "Cromwell for (%g %g %g)(%g %g %g) failed.\n", a11, a22, a33, a12, a13, a23));
        M.identity(); Jacobi3(M);
    }
};

/*######################################################################*/

class eps_t {
    double m_tolerance, m_clearance;
    double m_eps, m_el, m_clr, m_clr2;

public:
    double getEps(const vec_t &v) const {
        return m_clr ? m_clr * v.norm() : -m_eps * v.absnorm();
    }
    bool twoSides(double x, double y) const {
        return (x < m_el && y > -m_el) || (y < m_el && x > -m_el);
    }
    void setWorld(double scale, double unit) {
        m_clr = m_clearance / scale; m_clr2 = m_clr * m_clr;
        m_el = EPS0 * unit; m_eps = m_tolerance * unit;
    }
    bool hasClearance() const { return m_clr > 0; }
    bool ltUnit(double x) const { return x < m_el; }
    bool gtUnit(double x) const { return x > m_el; }
    bool cleared(double x) const { return x > m_clr2; }
    bool cleared(double x, double y) const { return rabs(x) > m_clr + y; }
    double square(double x) const { return rsq(x + m_clr); }
    void setTolerance(double tol) { m_tolerance = (double) tol; }
    void setClearance(double clr) { m_clearance = (double) clr; }
    eps_t() : m_tolerance(EPS0), m_clearance(0.0) {}
};

/*######################################################################*/

class tlist_t {
    int     m_size; // Number of triangles.
    trgs_t *m_root; // The triangle list root.

public:
    void addTrgs(vec_t &mean) {
        for (trgs_t *t=m_root; t; t=t->next)
            for (int j=0; j<3; j++) mean += t->t[j];
    }
    void append(const double *const *trg) {
        append(new trgs_t);
        m_root->t[0].init(trg[0]);
        m_root->t[1].init(trg[1]);
        m_root->t[2].init(trg[2]);
    }
    void clearTrgs() { m_root = 0; }
    int size() const { return m_size; }
    trgs_t *getTrgs() { return m_root; }
    void append(trgs_t *t) { t->next = m_root; m_root = t; m_size++; }
    const trgs_t *getTrgs() const { return m_root; }
    tlist_t() : m_size(0), m_root(0) {}
    ~tlist_t() { for (trgs_t *t=m_root, *tn; t; t=tn) { tn=t->next; delete t; } }
};

/*######################################################################*/

class obj_t {
    tlist_t m_trgs;    // Triangles.
    obj_t *m_left;    // Left child.
    obj_t *m_right;   // Right child.
    double  m_radius0; // Inner size.
    double  m_radius1; // Outer size.
    xform_t m_xform;   // Transformation to world.
    vec_t   m_dim;     // Dimension.

    void split(const vec_t &mean) {
        if (size() < 4) return; // No split under this size.

        // Find axis of largest dimension.
        int a = 0; if (m_dim[1] > m_dim[a]) a = 1; if (m_dim[2] > m_dim[a]) a = 2;

        if (m_dim[a] == 0.0) return; // Impossible to split.

        m_left = new obj_t; m_right = new obj_t; // Create subtrees.

        double mp = 3 * mean.dot(m_xform[a]), eps = m_dim[a] / 30;

        // Partition accordingly ...
        for (trgs_t *t=m_trgs.getTrgs(), *tn; t; t=tn) {
            tn = t->next;
            double p = t->t[0].dot(m_xform[a]) + t->t[1].dot(m_xform[a]) + t->t[2].dot(m_xform[a]) - mp;
            (p > eps || (p > -eps && m_left->size()) < m_right->size() ? m_left : m_right)->m_trgs.append(t);
        }

        if (m_left->size() && m_right->size()) { m_trgs.clearTrgs(); return; }

        // Abort the split if either l or r are empty.
        dbg((stderr, "Split failed: axis = %d, n = %d, l->n = %d,  r->n = %d\n",
             a, size(), m_left->size(), m_right->size()));
        delete m_left; delete m_right; m_left = m_right = 0;
    }
    void refit() {
        // Find max and min along each axis.
        vec_t Min = m_xform.multT(getTrgs()->t[0]), Max = Min;

        for (const trgs_t *t=getTrgs(); t; t=t->next)
            for (int i=0; i<3; i++)
                for (int j=0; j<3; j++) {
                    double v = t->t[j].dot(m_xform[i]);
                    if (v < Min[i]) Min[i] = v; else if (v > Max[i]) Max[i] = v;
                }

        // Place the origin in the proper position.
        m_xform.t = m_xform.mult((Max + Min) * 0.5);

        // Compute dimension.
        m_dim = (Max - Min) * 0.5;
        m_radius0 = rmin(m_dim[0], m_dim[1], m_dim[2]);
        m_radius1 = m_dim.norm();
    }

public:
    void translate(vec_t &t, const xform_t &X21, const obj_t *obj) const {
        t = X21.mult(obj->m_xform.t) + X21.t - m_xform.t;
    }
    void orient(xform_t &B, const mat_t &M21, const obj_t *obj) const {
        mat_t T; B.t = m_xform.multT(B.t); T.mult(obj->m_xform, M21); B.multT(T, m_xform);
    }
    void build(bool orientBBox, bool all = false) {
        if (!getTrgs() || m_radius1 >= 0) return;

        // Compute center of mass.
        vec_t mean; mean.zero(); m_trgs.addTrgs(mean);
        mean *= (double)(1.0/3.0/m_trgs.size());

        // Returns a new orientation matrix which better fits the data.
        if (orientBBox) { orient_t o(mean, getTrgs(), m_xform); } else m_xform.identity();

        // Assign position and extent ...
        refit(); split(mean);

        // ... and build children.
        if (!all) return;
        if (m_left) m_left->build(orientBBox, all);
        if (m_right) m_right->build(orientBBox, all);
    }
    int append(const double *v1, const double *v2, const double *v3) {
            const double *trg[] = {v1, v2, v3}; return append(trg);
    }
    int append(const float *const trg[3]) {
        double v1[] = {trg[0][0], trg[0][1], trg[0][2]};
        double v2[] = {trg[1][0], trg[1][1], trg[1][2]};
        double v3[] = {trg[2][0], trg[2][1], trg[2][2]};
        return append(v1, v2, v3);
    }
    int append(const float *v1, const float *v2, const float *v3) {
        const float *trg[] = {v1, v2, v3}; return append(trg);
    }
    obj_t *getLeft() { return m_left; }
    obj_t *getRight() { return m_right; }
    double getRadius0() const { return m_radius0; }
    double getRadius1() const { return m_radius1; }
    double getDim(int i) const { return m_dim[i]; }
    const trgs_t *getTrgs() const { return m_trgs.getTrgs(); }
    virtual int size() const { return m_trgs.size(); }
    virtual int destroy() { delete this; return 0; }
    virtual int append(const double *const *trg) { m_trgs.append(trg); return 0; }
    obj_t() : m_left(0), m_right(0), m_radius1(-1.0) {}
    virtual ~obj_t() { if (m_left) delete m_left; if (m_right) delete m_right; }
};

/*######################################################################*/

struct callback_t {
    /* return false to discontinue notifications */
    virtual bool point(const double point[3]) = 0;
    virtual bool line(const double line[2][3]) = 0;
    virtual ~callback_t() {}
};

class collide_t {
    bool    m_first;
    bool    m_orient;
    eps_t   m_eps;
    double  m_s21;
    mat_t   m_M21;
    xform_t m_X1, m_X21;
    callback_t *m_callback;
    int m_collides, m_boxTests, m_trgTests;

    bool xTrgPlane(vec_t *p, double *f, const trg_t &t, double d, const vec_t &n, const vec_t &v) {
        int x = 0;
        double pn[] = {d - n.dot(t[0]), d - n.dot(t[1]), d - n.dot(t[2])};
        for (int e = 1; e < 4 && x < 2; e++) {
            int i = e == 3, j = e - i; double dp = pn[j] - pn[i];
            if (m_eps.ltUnit(rabs(dp)) || !m_eps.twoSides(pn[i], pn[j])) continue;
            double a = pn[j] / dp; p[x] = (t[i] - t[j]) * a; p[x] += t[j];
            if (!x || m_eps.gtUnit((p[0] - p[1]).absnorm())) x++;
        }
        if (x < 2) return false; f[0] = v.dot(p[0]); f[1] = v.dot(p[1]); return true;
    }
    bool isSeparatingPlane(const vec_t &v1, const vec_t &v2, const trg_t &t1, const trg_t &t2) {
        vec_t normal(v1, v2); double e = m_eps.getEps(normal);
        double P1 = normal.dot(t1[0]), P2 = normal.dot(t1[1]), P3 = normal.dot(t1[2]);
        double Q1 = normal.dot(t2[0]), Q2 = normal.dot(t2[1]), Q3 = normal.dot(t2[2]);
        return rmin(P1, P2, P3) - rmax(Q1, Q2, Q3) > e || rmin(Q1, Q2, Q3) - rmax(P1, P2, P3) > e;
    }
    void collideTriangles(const trgs_t *trg1, const trgs_t *trg2) {
        for (const trgs_t *t1=trg1; t1; t1=t1->next)
            for (const trgs_t *t2=trg2; t2; t2=t2->next) {
                // Transform triangle from b2 into b1.
                trg_t T2; T2.mult(t2->t, m_X21); T2 += m_X21.t;
                intersectTriangles(t1->t, T2); if (m_first && m_collides) return;
            }
    }
    void getIntersection(const trg_t &t10, const trg_t &t20) {
        trg_t t1, t2; vec_t p[2][2]; double s, f[2][2];
        t1.mult(t10, m_X1); t1 += m_X1.t;
        t2.mult(t20, m_X1); t2 += m_X1.t;

        vec_t n1(t1[1] - t1[0], t1[2] - t1[0]); if (m_eps.ltUnit(s = n1.absnorm())) return; n1 *= 1/s;
        vec_t n2(t2[1] - t2[0], t2[2] - t2[0]); if (m_eps.ltUnit(s = n2.absnorm())) return; n2 *= 1/s;

        vec_t v(n1, n2); if (v.norm2() < EPS0) return;
        if (!xTrgPlane(p[0], f[0], t1, t2[0].dot(n2), n2, v)) return;
        if (!xTrgPlane(p[1], f[1], t2, t1[0].dot(n1), n1, v)) return;

        int i[2]; i[0] = f[0][1] < f[0][0]; i[1] = f[1][1] < f[1][0];
        int l0 = f[1][i[1]] > f[0][i[0]], l1 = f[1][1 - i[1]] < f[0][1 - i[0]];
        if (f[l0][i[l0]] > f[l1][1 - i[l1]]) return;

        double line[2][3];
        p[l0][i[l0]].get(line[0]);
        p[l1][1 - i[l1]].get(line[1]);
        m_callback->line(line);
    }
    static double distance(const trg_t &trg1, const trg_t &trg2);
    void intersectTriangles(const trg_t &t1, const trg_t &t2) {
        m_trgTests++;
        vec_t e1 = t1[1] - t1[0], e2 = t1[2] - t1[1], e3 = t1[0] - t1[2];
        vec_t f1 = t2[1] - t2[0], f2 = t2[2] - t2[1], f3 = t2[0] - t2[2];

        if (isSeparatingPlane(e1, e2, t1, t2)) return;
        if (isSeparatingPlane(f1, f2, t1, t2)) return;

        if (isSeparatingPlane(e1, f1, t1, t2)) return;
        if (isSeparatingPlane(e2, f2, t1, t2)) return;
        if (isSeparatingPlane(e3, f3, t1, t2)) return;
        if (isSeparatingPlane(e1, f2, t1, t2)) return;
        if (isSeparatingPlane(e1, f3, t1, t2)) return;
        if (isSeparatingPlane(e2, f3, t1, t2)) return;
        if (isSeparatingPlane(e2, f1, t1, t2)) return;
        if (isSeparatingPlane(e3, f1, t1, t2)) return;
        if (isSeparatingPlane(e3, f2, t1, t2)) return;

        if (m_eps.hasClearance() && m_eps.cleared(distance(t1, t2))) return;
        if (m_callback) getIntersection(t1, t2); m_collides++;
    }
    bool disjointBoxes(const obj_t *obb1, const obj_t *obb2) {
        m_boxTests++; xform_t BB; obb1->translate(BB.t, m_X21, obb2); double d = BB.t.norm2();
        if (m_eps.square(obb1->getRadius0() + m_s21 * obb2->getRadius0()) > d) return false;
        if (m_eps.square(obb1->getRadius1() + m_s21 * obb2->getRadius1()) < d) return true;

        if (m_orient) obb1->orient(BB, m_M21, obb2); else *(mat_t *)&BB = m_M21;
        double f11 = rabs(BB[0][0]), f12 = rabs(BB[0][1]), f13 = rabs(BB[0][2]);
        double f21 = rabs(BB[1][0]), f22 = rabs(BB[1][1]), f23 = rabs(BB[1][2]);
        double f31 = rabs(BB[2][0]), f32 = rabs(BB[2][1]), f33 = rabs(BB[2][2]);

        double a1 = obb1->getDim(0), a2 = obb1->getDim(1), a3 = obb1->getDim(2);
        double b1 = m_s21*obb2->getDim(0), b2 = m_s21*obb2->getDim(1), b3 = m_s21*obb2->getDim(2);

        // A1 x A2 = A0, A2 x A0 = A1, A0 x A1 = A2
        if (m_eps.cleared(BB.t[2], a3 + b1*f13 + b2*f23 + b3*f33)) return true;
        if (m_eps.cleared(BB.t[1], a2 + b1*f12 + b2*f22 + b3*f32)) return true;
        if (m_eps.cleared(BB.t[0], a1 + b1*f11 + b2*f21 + b3*f31)) return true;

        // B1 x B2 = B0, B2 x B0 = B1, B0 x B1 = B2
        if (m_eps.cleared(BB.t.dot(BB[2]), b3 + a1*f31 + a2*f32 + a3*f33)) return true;
        if (m_eps.cleared(BB.t.dot(BB[1]), b2 + a1*f21 + a2*f22 + a3*f23)) return true;
        if (m_eps.cleared(BB.t.dot(BB[0]), b1 + a1*f11 + a2*f12 + a3*f13)) return true;

        // A0 x B0, A0 x B1, A0 x B2
        if (m_eps.cleared(BB.t[2]*BB[0][1] - BB.t[1]*BB[0][2], a2*f13 + a3*f12 + b2*f31 + b3*f21)) return true;
        if (m_eps.cleared(BB.t[2]*BB[1][1] - BB.t[1]*BB[1][2], a2*f23 + a3*f22 + b1*f31 + b3*f11)) return true;
        if (m_eps.cleared(BB.t[2]*BB[2][1] - BB.t[1]*BB[2][2], a2*f33 + a3*f32 + b1*f21 + b2*f11)) return true;

        // A1 x B0, A1 x B1, A1 x B2
        if (m_eps.cleared(BB.t[0]*BB[0][2] - BB.t[2]*BB[0][0], a1*f13 + a3*f11 + b2*f32 + b3*f22)) return true;
        if (m_eps.cleared(BB.t[0]*BB[1][2] - BB.t[2]*BB[1][0], a1*f23 + a3*f21 + b1*f32 + b3*f12)) return true;
        if (m_eps.cleared(BB.t[0]*BB[2][2] - BB.t[2]*BB[2][0], a1*f33 + a3*f31 + b1*f22 + b2*f12)) return true;

        // A2 x B0, A2 x B1, A2 x B2
        if (m_eps.cleared(BB.t[1]*BB[0][0] - BB.t[0]*BB[0][1], a1*f12 + a2*f11 + b2*f33 + b3*f23)) return true;
        if (m_eps.cleared(BB.t[1]*BB[1][0] - BB.t[0]*BB[1][1], a1*f22 + a2*f21 + b1*f33 + b3*f13)) return true;
        if (m_eps.cleared(BB.t[1]*BB[2][0] - BB.t[0]*BB[2][1], a1*f32 + a2*f31 + b1*f23 + b2*f13)) return true;
        return false;
    }
    void collide(obj_t *obb1, obj_t *obb2) {
        if (m_first && m_collides) return;
        obb1->build(m_orient); obb2->build(m_orient);

        // Check boxes ...
        if (disjointBoxes(obb1, obb2)) return;

        // A leaf pair ? - check triangles.
        if (!obb1->getLeft() && !obb2->getLeft()) { collideTriangles(obb1->getTrgs(), obb2->getTrgs()); return; }

        // Descend to children.
        if (!obb2->getLeft() || (obb1->getLeft() && obb1->getRadius1()) > obb2->getRadius1()) {
            collide(obb1->getLeft(), obb2);
            collide(obb1->getRight(), obb2);
        } else {
            collide(obb1, obb2->getLeft());
            collide(obb1, obb2->getRight());
        }
    }
    void collide(const xform_t &x1, double s1, obj_t *obb1, const xform_t &x2, double s2, obj_t *obb2) {
        // Reset data.
        m_collides = m_boxTests = m_trgTests = 0;
        if (!obb1->size() || !obb2->size()) return;

        obb1->build(m_orient); obb2->build(m_orient);
        m_eps.setWorld(s1, rmin(obb1->getRadius1(), obb2->getRadius1()));

        // From obb1 world to global world.
        m_X1 = x1; m_X1 *= s1;

        // From obb2 world to obb1 world.
        m_X21.multT(x2, x1); m_s21 = s2/s1;
        m_X21.t = x1.multT(x2.t - x1.t) * (1/s1);
        m_M21 = m_X21; m_X21 *= m_s21;
        collide(obb1, obb2);
    }

public:
    virtual int setOrientBBox(bool flag) { m_orient = flag; return 0; }
    virtual int setTolerance(double eps) { m_eps.setTolerance(eps); return 0; }
    virtual int setClearance(double clr) { m_eps.setClearance(clr); return 0; }

    virtual int createBox(obj_t *&box) { box = new obj_t; return 0; }
    virtual int build(obj_t *box, bool all = true) { if (box) ((obj_t *) box)->build(m_orient, all); return 0; }

    virtual int check(const double *xform1, obj_t *b1, const double *xform2, obj_t *b2, bool first = true) {
        m_first = first;
        xform_t x1(xform1); double s1 = x1[0].norm(); x1 *= 1/s1;
        xform_t x2(xform2); double s2 = x2[0].norm(); x2 *= 1/s2;
        collide(x1, s1, (obj_t*) b1, x2, s2, (obj_t*) b2);
        return m_collides;
    }
    virtual int check(const double *xform1, obj_t *b1, const double ray[2][3], bool closest = true) {
        double point[3]; const double *trg[] = {ray[0], ray[1], point};
        point[0] = ray[1][0] + 0.01; point[1] = ray[1][1]; point[2] = ray[1][2];
        obj_t b2; b2.append(trg); b2.build(m_orient, true);
        m_first = closest;
        xform_t x1(xform1); double s1 = x1[0].norm(); x1 *= 1/s1;
        collide(x1, s1, (obj_t*) b1, 0, (double)1.0, &b2);
        return m_collides;
    }
    static int create(collide_t *&collide) {
        collide = new collide_t; return collide ? 0 : -1;
    }
    void setCallback(callback_t *callback) { m_callback = callback; }
    virtual int destroy() { delete this; return 0; }
    collide_t() : m_orient(true) {}
    virtual ~collide_t() {}
};

inline double collide_t::distance(const trg_t &trg1, const trg_t &trg2)
{
    // TODO ...
    return -1;
}
