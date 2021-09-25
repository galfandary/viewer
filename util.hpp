#ifdef NO_NAMESPACE
# define NAMESPACE_BEGIN(ns)
# define NAMESPACE_END(ns)
# define NAMESPACE_USING(ns)
#else
# define NAMESPACE_BEGIN(ns) namespace ns {
# define NAMESPACE_END(ns) }
# define NAMESPACE_USING(ns) using namespace ns
#endif

#define STATIC_START namespace {
#define STATIC_END }

#define OBJ_ZERO(x) memset(&x, 0, sizeof(x))
#define OBJ_PZERO(x) memset(x, 0, sizeof(*x))
#define ARR_SIZE(arr) (sizeof(arr)/sizeof(arr[0]))

/*######################################################################*/

template<typename obj_t> class pointers_t {
    obj_t **A;
    int X, mem;
    float growth;
    void operator=(const pointers_t &){}
public:
    void resize(int n) {
        X = n;
        if (n <= mem) return;
        mem = (int) (growth * n);
        A = (obj_t **) realloc(A, mem * sizeof(A[0]));
    }
    void insert(obj_t *obj, int i) {
        int s = X; expand();
        obj_t **from = A + s, **to = from--, **end = A + i;
        while (to > end) *to-- = *from--;
        *end = obj;
    }
    void remove(const obj_t *obj) {
        int i = 0;
        while (i < X && obj != at(i)) i++;
        for (X--; i < X; i++) at(i) = at(i + 1);
    }
    int size() const { return X; }
    obj_t *&at(int i) { return A[i]; }
    obj_t *&operator[](int i) { return at(i); }
    obj_t **const array() const { return A; }
    bool ok(int i) const { return 0 <= i && i < X; }
    void expand(int n = 1) { resize(X + n); }
    void append(obj_t *obj) { expand(); A[X - 1] = obj; }
    void eraseObjects() { while (X) delete A[--X]; }
    const obj_t *at(int i) const { return A[i]; }
    const obj_t *operator[](int i) const { return at(i); }
    pointers_t(int s = 0, float g = 1.2f) : A(0), X(s), mem(0), growth(g) {
        if (X) A = (obj_t **) malloc(X * sizeof(A[0]));
        if (A) mem = X;
    }
    ~pointers_t() { if (A) free(A); }
};

/*######################################################################*/

template<typename obj_t> class smap_t {
    struct ent_t {char *key; obj_t obj; ~ent_t(){free(key);}};
    pointers_t<ent_t> m_arr;
    obj_t *insert(const char *key, int i) {
        ent_t *ent = new ent_t;
        m_arr.insert(ent, i);
        ent->key = strdup(key); return &ent->obj;
    }
    static int find(const char *key, const char ***arr, int i, int j) {
        if (i >= j) return j; int k = (i + j) / 2;
        return strcmp(key, *arr[k]) > 0 ? find(key, arr, ++k, j) : find(key, arr, i, k);
    }
    int index(const char *key) const {
        return size() ? find(key, (const char ***)m_arr.array(), 0, size()) : 0;
    }
public:
    obj_t *add(const char *key) {
        int i = index(key); obj_t *obj = i < size() ? pobjAt(i) : 0;
        return !obj || strcmp(m_arr[i]->key, key) ? insert(key, i) : obj;
    }
    obj_t *add(const char *key, const obj_t &value) {
        obj_t *obj = add(key); if (obj) *obj = value; return obj;
    }
    obj_t *find(const char *key) {
        int i = index(key); obj_t *obj = i < size() ? pobjAt(i) : 0;
        return !obj || strcmp(m_arr[i]->key, key) ? 0 : obj;
    }
    const obj_t *find(const char *key) const {
        int i = index(key); const obj_t *obj = i < size() ? pobjAt(i) : 0;
        return !obj || strcmp(m_arr[i]->key, key) ? 0 : obj;
    }
    void eraseObjects() {
        for (int i = 0, n = size(); i < n; i++) delete m_arr[i]->obj;
    }
    int size() const { return m_arr.size(); }
    obj_t &objAt(int i) { return m_arr[i]->obj; }
    obj_t *pobjAt(int i) { return &m_arr[i]->obj; }
    const char *keyAt(int i) const { return m_arr[i]->key; }
    const obj_t &objAt(int i) const { return m_arr[i]->obj; }
    const obj_t *pobjAt(int i) const { return &m_arr[i]->obj; }
    ~smap_t() { m_arr.eraseObjects(); }
};

/*######################################################################*/

struct gui_state_t {
    bool alt:1;
    bool shift:1;
    bool control:1;
    bool mouse1:1;
    bool mouse2:1;
    bool mouse3:1;
    gui_state_t() { OBJ_PZERO(this); }
};

struct gui_data_t {
    void *dpy;
    const char *title;
    int width;
    int height;
    int xoffset;
    int yoffset;
    bool colorIndex:1;
    bool singleBuffer:1;
};

struct gui_viewer_t {
    virtual void wheel(bool up, const gui_state_t &state) = 0;
    virtual void move(int x, int y, const gui_state_t &state) = 0;
    virtual void press(int key, const gui_state_t &state) = 0;
    virtual void press(int mouse, int x, int y, const gui_state_t &state) = 0;
    virtual void release(int mouse) = 0;
    virtual void resize(int w, int h) = 0;
    virtual void repaint() = 0;
protected:
    virtual ~gui_viewer_t() {}
};

/*######################################################################*/
