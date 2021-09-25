struct texture_t {
    float     percent;
    char      file[64];
};

struct material_t {
    float     shininess;
    float     ambient[4];
    float     diffuse[4];
    float     specular[4];
    texture_t texture;
    texture_t reflect;
    bool      twoside;
};

struct geometry_t {
    float    *vertex[3];
    float    *normal[3];
    float    *texture[3];
};

struct track_t {
    int       size;
    float     array[1024];
};

struct ds_node_t {
    int       id;
    int       parent;
    char      object[256];
    float     pivot[3];
    track_t   scale;
    track_t   rotate;
    track_t   translate;
};

struct reader_t {
    virtual void nodeCB(const ds_node_t *node) = 0;
    virtual void objectCB(const char *name) = 0;
    virtual void matrixCB(const float matrix[4][3]) = 0;
    virtual void materialCB(const material_t *material) = 0;
    virtual void triangleCB(const geometry_t *geometry) = 0;
protected:
    virtual ~reader_t() {}
};

/*######################################################################*/

typedef uint8_t  u1_t;
typedef int16_t  i2_t;
typedef uint16_t u2_t;
typedef int32_t  i4_t;
typedef uint32_t u4_t;
typedef float    r4_t;
typedef long     po_t;

const u2_t uSed = 0x1000;
const u4_t uErr = (u4_t) -1;

/*######################################################################*/

struct map_t : private smap_t<material_t*> {
    void setMaterial(const char *name, material_t *material) { add(name, material); }
    const material_t *getMaterial(const char *name) const {
        const material_t *const *m = find(name); return m ? *m : 0;
    }
    ~map_t() { eraseObjects(); }
};
struct matgroup_t {
    u2_t n, *arr;
    const material_t *material;
    u2_t *create() { return arr = new u2_t[n]; }
    bool transparent() { return material->ambient[3] < 1.0f; }
    matgroup_t() : n(0), arr(0) {}
    ~matgroup_t() { if (arr) delete[] arr; }
};
struct matgroups_t : private pointers_t<matgroup_t> {
    int getSize() { return size(); }
    matgroup_t *get(int i) { return at(i); }
    void add(matgroup_t *matgroup) { append(matgroup); }
    void sort() {
        int i = 0, j = size() - 1;
        while (i < j) {
            while (i < j && !at(i)->transparent()) i++;
            while (i < j && at(j)->transparent()) j--;
            if (i < j) { matgroup_t *t = at(i); at(i) = at(j); at(j) = t; }
        }
    }
    ~matgroups_t() { eraseObjects(); }
};

/*######################################################################*/

inline r4_t zx(r4_t x) {
    return fabs(x) < 1E-4 ? 0 : x;
}
inline void printMatrix(const r4_t matrix[4][3]) {
#   ifdef DEBUG_3DS
    printf("Matrix\n");
    for (int i = 0; i < 4; i++)
        printf("%g %g %g\n", zx(matrix[i][0]), zx(matrix[i][1]), zx(matrix[i][2]));
#   endif
}
inline void identity(r4_t matrix[4][3]) {
    memset(matrix, 0, 48);
    matrix[0][0] = matrix[1][1] = matrix[2][2] = 1;
}
inline void copy(r4_t *to, const r4_t *from) {
    memcpy(to, from, 12);
}
inline void scale(r4_t *v, r4_t f) {
    *v++ *= f; *v++ *= f; *v++ *= f;
}
inline void add(r4_t *v, const r4_t *v1) {
    *v++ += *v1++; *v++ += *v1++; *v++ += *v1++;
}
inline void minus(r4_t *v, const r4_t *v1) {
    *v++ -= *v1++; *v++ -= *v1++; *v++ -= *v1++;
}
inline void add(r4_t *v, const r4_t *v1, const r4_t *v2) {
    *v++ = *v1++ + *v2++; *v++ = *v1++ + *v2++; *v++ = *v1++ + *v2++;
}
inline void minus(r4_t *v, const r4_t *v1, const r4_t *v2) {
    *v++ = *v1++ - *v2++; *v++ = *v1++ - *v2++; *v++ = *v1++ - *v2++;
}
inline int unit(r4_t *v) {
    r4_t r = (r4_t) sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (r < 1E-5) return -1;
    *v++ /= r; *v++ /= r; *v++ /= r; return 0;
}
inline void cross(r4_t *v, const r4_t *v1, const r4_t *v2) {
    *v++ = v1[1] * v2[2] - v1[2] * v2[1];
    *v++ = v1[2] * v2[0] - v1[0] * v2[2];
    *v++ = v1[0] * v2[1] - v1[1] * v2[0];
}

/*######################################################################*/

class io_t {
    FILE *m_fp;
    map_t m_map;
    reader_t *m_reader;
public:
    int get(void *x, u4_t n) { return fread(x, 1, n, m_fp) == n ? 0 : -1; }
    int get(u1_t &x) { return get(&x, sizeof(x)); }
    int get(i2_t &x) { return get(&x, sizeof(x)); }
    int get(u2_t &x) { return get(&x, sizeof(x)); }
    int get(i4_t &x) { return get(&x, sizeof(x)); }
    int get(u4_t &x) { return get(&x, sizeof(x)); }
    int get(r4_t &x) { return get(&x, sizeof(x)); }
    int slot(u1_t &x) { u4_t n; return get(n) || get(x) ? -1 : 0; }
    int slot(i2_t &x) { u4_t n; return get(n) || get(x) ? -1 : 0; }
    int slot(u2_t &x) { u4_t n; return get(n) || get(x) ? -1 : 0; }
    int slot(i4_t &x) { u4_t n; return get(n) || get(x) ? -1 : 0; }
    int slot(u4_t &x) { u4_t n; return get(n) || get(x) ? -1 : 0; }
    int slot(r4_t &x) { u4_t n; return get(n) || get(x) ? -1 : 0; }
    int slot(void *x) { u4_t n; return get(n) || get(x, n - 6) ? -1 : 0; }
    int pos(po_t &p) { p = ftell(m_fp); return p < 0 ? -1 : 0; }
    int move(po_t p) { return fseek(m_fp, p, SEEK_SET) ? -1 : 0; }
    int skip() { po_t e; return end(e) || move(e) ? -1 : 0; }
    int ignore(const char *child) { return skip(); }
    int unknownChild(u2_t child) { return skip(); }
    u2_t getChild(u2_t &child) { return get(child) ? uErr : child; }
    bool ok(po_t &e) { po_t p; return !pos(p) && p < e; }
    int end(po_t &p) {
        u4_t n; if (get(n) || pos(p)) return -1;
        p += n - 6; return 0;
    }
    int readVersion() {
        i4_t version; if (slot(version)) return -1;
        return 0;
    }
    int readZString(char *str) {
        int r = 0; u1_t c;
        while (!(r += get(c)) && (*str++ = c));
        return r;
    }
    int readZStringSlot(char *str) {
        po_t e; if (end(e)) return -1;
        return readZString(str);
    }
    int readIntPercentage(r4_t &percentage) {
        i2_t p; if (slot(p)) return -1;
        percentage = p * 0.01f; return 0;
    }
    int readPercentage(r4_t &percentage) {
        u2_t child; po_t e; if (end(e)) return -1;
        while (ok(e)) {
            switch (getChild(child)) {
            case 0x0030: if (readIntPercentage(percentage)) return -1; continue;
            case 0x0031: if (slot(percentage)) return -1; continue;
            default: unknownChild(child); continue;
            }
        }
        return 0;
    }
    void rgb2color(float color[3], const u1_t rgb[3]) {
        const float f = 1.0f / 255.0f;
        color[0] = f * rgb[0]; color[1] = f * rgb[1]; color[2] = f * rgb[2];
    }
    int readColor(float color[3]) {
        u1_t rgb[3]; u2_t child; po_t e; if (end(e)) return -1;
        while (ok(e)) {
            switch (getChild(child)) {
            case 0x0011: if (slot(rgb)) return -1; continue;
            case 0x0012: if (slot(rgb)) return -1; continue;
            default: unknownChild(child); continue;
            }
        }
        rgb2color(color, rgb); return 0;
    }
    int readArray3D(r4_t *&arr, u2_t &n) {
        po_t e; if (end(e) || get(n)) return -1;
        return get(arr = new r4_t[3 * n], 12 * n);
    }
    int readArray2D(r4_t *&arr, u2_t &n) {
        po_t e; if (end(e) || get(n)) return -1;
        return get(arr = new r4_t[2 * n], 8 * n);
    }
    int readSmoothGroup(u4_t *&arr, u4_t n) {
        po_t e; if (end(e)) return -1;
        return get(arr = new u4_t[n], 4 * n);
    }
    int readMatGroup(matgroups_t &matgroups) {
        char name[64]; if (readZStringSlot(name)) return -1;
        matgroup_t *matgroup = new matgroup_t;
        matgroup->material = m_map.getMaterial(name); matgroups.add(matgroup);
        int r = get(matgroup->n) || get(matgroup->create(), 2 * matgroup->n) ? -1 : 0;
        return r;
    }
    int readFaceArray(matgroups_t &matgroups, u2_t *&faces, u4_t *&smoothing, u2_t &n) {
        u2_t child; po_t e; if (end(e) || get(n)) return -1;
        if (get(faces = new u2_t[4 * n], 8 * n)) return -1;
        while (ok(e)) {
            switch (getChild(child)) {
            case 0x4130: if (readMatGroup(matgroups)) return -1; continue;
            case 0x4150: if (readSmoothGroup(smoothing, n)) return -1; continue;
            case 0x4190: if (ignore("MSH_BOXMAP")) return -1; continue;
            default: unknownChild(child); continue;
            }
        }
        return 0;
    }
    r4_t *mainNormals(r4_t *a3d, u4_t n3d, u2_t *faces, u4_t n) {
        r4_t *mains = new r4_t[3 * n];
        for (u4_t i = 0; i < n; i++) {
            r4_t v1[3], v2[3], normal[3];
            u2_t *face = faces + 4 * i;
            bool ok = face[0] < n3d && face[1] < n3d && face[2] < n3d;
            if (ok) {
                minus(v1, a3d + 3 * face[1], a3d + 3 * face[0]);
                minus(v2, a3d + 3 * face[2], a3d + 3 * face[0]);
                cross(normal, v1, v2);
            }
            if (!ok || unit(normal)) face[3] |= uSed; else copy(mains + 3 * i, normal);
        }
        return mains;
    }
    struct face_t { u4_t face; face_t *next; };
    r4_t *calculateNormals(r4_t *a3d, u4_t n3d, u2_t *faces, u4_t nf, u4_t *smoothing) {
        u4_t i, j, nf3 = 3 * nf;
        face_t *fa = new face_t[nf3], **fl = new face_t*[n3d];
        r4_t *normals = new r4_t[3*nf3], *mains = mainNormals(a3d, n3d, faces, nf);
        for (i = 0; i < nf3; i++) fa[i].next = 0;
        for (i = 0; i < n3d; i++) fl[i] = 0;
        for (i = 0; i < nf; i++)
            for (j = 0; j < 3; j++) {
                u4_t v = faces[4 * i + j];
                if (v >= n3d) continue;
                face_t *l = fa + 3 * i + j;
                l->face = i; l->next = fl[v]; fl[v] = l;
            }
        for (i = 0; i < nf; i++) {
            if (!smoothing || !smoothing[i]) {
                copy(normals + i * 9 + 0, mains + i * 3);
                copy(normals + i * 9 + 3, mains + i * 3);
                copy(normals + i * 9 + 6, mains + i * 3);
                continue;
            }
            for (j = 0; j < 3; j++) {
                r4_t n[] = {0.0, 0.0, 0.0};
                u4_t v = faces[4 * i + j];
                if (v >= n3d) continue;
                for (face_t *p = fl[v]; p; p = p->next)
                    if (!(faces[4 * p->face + 3] & uSed) && (smoothing[i] & smoothing[p->face]))
                        add(n, mains + 3 * p->face);
                copy(normals + i * 9 + j * 3, unit(n) ? mains + i * 3 : n);
            }
        }
        delete[] fa;
        delete[] fl;
        delete[] mains;
        return normals;
    }
    void triangleCB(r4_t *a3d, r4_t *a2d, r4_t *normals, u2_t *face) {
        if (!m_reader || face[3] & uSed) return; face[3] |= uSed;
        geometry_t geometry = {
            {a3d + 3 * face[0], a3d + 3 * face[1], a3d + 3 * face[2]},
            {normals, normals + 3, normals + 6},
            {a2d + 2 * face[0], a2d + 2 * face[1], a2d + 2 * face[2]}
        };
        if (!a2d) geometry.texture[0] = geometry.texture[1] = geometry.texture[2] = 0;
        m_reader->triangleCB(&geometry);
    }
    int readTriObject() {
        po_t e; if (end(e)) return -1;
        u1_t color = 0;
        u4_t *smoothing = 0;
        r4_t *a2d = 0, *a3d = 0, matrix[4][3];
        u2_t *faces = 0, child, nf = 0, n2d = 0, n3d = 0;
        matgroups_t matgroups;
        identity(matrix);
        while (ok(e)) {
            switch (getChild(child)) {
            case 0x4110: if (readArray3D(a3d, n3d)) return -1; continue;
            case 0x4120: if (readFaceArray(matgroups, faces, smoothing, nf)) return -1; continue;
            case 0x4140: if (readArray2D(a2d, n2d)) return -1; continue;
            case 0x4160: if (slot(matrix)) return -1; continue;
            case 0x4165: if (slot(color)) return -1; continue;
            case 0x4111: if (ignore("POINT_FLAG_ARRAY")) return -1; continue;
            default: unknownChild(child); continue;
            }
        }
        printMatrix(matrix);
        if (m_reader) m_reader->matrixCB(matrix);
        if (n3d && n2d && n3d != n2d) fprintf(stderr, "XXXX: %d != %d\n", n3d, n2d);
        matgroups.sort();
        r4_t *normals = calculateNormals(a3d, n3d, faces, nf, smoothing);
        for (u4_t g = 0, ng = matgroups.getSize(); g < ng; g++) {
            matgroup_t *mg = matgroups.get(g);
            if (m_reader) m_reader->materialCB(mg->material);
            for (u4_t i = 0, n = mg->n; i < n; i++)
                triangleCB(a3d, a2d, normals + 9 * mg->arr[i], faces + 4 * mg->arr[i]);
        }
        if (m_reader) m_reader->materialCB(0);
        for (u4_t i = 0; i < nf; i++) triangleCB(a3d, a2d, normals + 9 * i, faces + 4 * i);
        if (a2d) delete[] a2d;
        if (a3d) delete[] a3d;
        if (faces) delete[] faces;
        if (normals) delete[] normals;
        if (smoothing) delete[] smoothing;
        return 0;
    }
    int readObject() {
        u2_t child; po_t e; if (end(e)) return -1;
        char name[64]; if (readZString(name)) return -1;
        if (m_reader) m_reader->objectCB(name);
        while (ok(e)) {
            switch (getChild(child)) {
            case 0x4100: if (readTriObject()) return -1; continue;
            case 0x4600: if (ignore("N_DIRECT_LIGHT")) return -1; continue;
            case 0x4700: if (ignore("N_CAMERA")) return -1; continue;
            default: unknownChild(child); continue;
            }
        }
        if (m_reader) m_reader->objectCB(0);
        return 0;
    }
    int readTexture(texture_t &texture) {
        u2_t child; po_t e; if (end(e)) return -1;
        while (ok(e)) {
            switch (getChild(child)) {
            case 0x0030: if (readIntPercentage(texture.percent)) return -1; continue;
            case 0x0031: if (slot(texture.percent)) return -1; continue;
            case 0xA300: if (slot(texture.file)) return -1; continue;
            case 0xA351: if (ignore("MAT_MAP_TILING")) return -1; continue;
            case 0xA353: if (ignore("MAT_MAP_TEXBLUR")) return -1; continue;
            case 0xA354: if (ignore("MAT_MAP_USCALE")) return -1; continue;
            case 0xA356: if (ignore("MAT_MAP_VSCALE")) return -1; continue;
            case 0xA358: if (ignore("MAT_MAP_UOFFSET")) return -1; continue;
            case 0xA35A: if (ignore("MAT_MAP_VOFFSET")) return -1; continue;
            default: unknownChild(child); continue;
            }
        }
        return 0;
    }
    int readMaterial() {
        u2_t child; po_t e; if (end(e)) return -1;
        char name[64]; material_t *material = new material_t;
        memset(material, 0, sizeof(material_t));
        r4_t shininess = 0.0f, shin2pct = 0.0f, transparency = 0.0f;
        while (ok(e)) {
            switch (getChild(child)) {
            case 0xA000: if (slot(name)) return -1; continue;
            case 0xA010: if (readColor(material->ambient)) return -1; continue;
            case 0xA020: if (readColor(material->diffuse)) return -1; continue;
            case 0xA030: if (readColor(material->specular)) return -1; continue;
            case 0xA040: if (readPercentage(shininess)) return -1; continue;
            case 0xA041: if (readPercentage(shin2pct)) return -1; continue;
            case 0xA050: if (readPercentage(transparency)) return -1; continue;
            case 0xA052: if (ignore("MAT_XPFALL")) return -1; continue;
            case 0xA053: if (ignore("MAT_REFBLUR")) return -1; continue;
            case 0xA081: if (skip()) return -1; material->twoside = true; continue;
            case 0xA083: if (ignore("MAT_ADDITIVE")) return -1; continue;
            case 0xA084: if (ignore("MAT_SELF_ILPCT")) return -1; continue;
            case 0xA085: if (ignore("MAT_WIRE")) return -1; continue;
            case 0xA087: if (ignore("MAT_WIRE_SIZE")) return -1; continue;
            case 0xA08A: if (ignore("MAT_XPFALLIN")) return -1; continue;
            case 0xA08C: if (ignore("MAT_PHONGSOFT")) return -1; continue;
            case 0xA08E: if (ignore("MAT_WIREABS")) return -1; continue;
            case 0xA100: if (ignore("MAT_SHADING")) return -1; continue;
            case 0xA200: if (readTexture(material->texture)) return -1; continue;
            case 0xA220: if (readTexture(material->reflect)) return -1; continue;
            case 0xA230: if (ignore("MAT_BUMPMAP")) return -1; continue;
            case 0xA250: if (ignore("MAT_USE_REFBLUR")) return -1; continue;
            case 0xA310: if (ignore("MAT_ACUBIC")) return -1; continue;
            default: unknownChild(child); continue;
            }
        }
        material->ambient[3] = material->diffuse[3] = material->specular[3] = 1.0f - transparency;
        material->shininess = shininess < 0.7f ? (r4_t) pow(2.0, 10.0*shininess) : 128.0f;
        m_map.setMaterial(name, material);
        return 0;
    }
    int readData() {
        r4_t scale;
        u2_t child; po_t e; if (end(e)) return -1;
        while (ok(e)) {
            switch (getChild(child)) {
            case 0x3D3E: if (readVersion()) return -1; continue;
            case 0xAFFF: if (readMaterial()) return -1; continue;
            case 0x0100: if (slot(scale)) return -1; continue;
            case 0x4000: if (readObject()) return -1; continue;
            case 0x7001: if (ignore("VIEWPORT_LAYOUT")) return -1; continue;
            case 0x1400: if (ignore("LO_SHADOW_BIAS")) return -1; continue;
            case 0x1420: if (ignore("SHADOW_MAP_SIZE")) return -1; continue;
            default: unknownChild(child); continue;
            }
        }
        return 0;
    }
    int readNodeHeader(char *name, u2_t &flags1, u2_t &flags2, u2_t &parent) {
        return readZStringSlot(name) || get(flags1) || get(flags2) || get(parent) ? -1 : 0;
    }
    int readTCB(const char *type, i4_t keys, u2_t flags, r4_t *arr, bool n4) {
        i4_t frame; u2_t f, flg;
        if (get(frame) || get(flg)) return -1;
        r4_t tension, continuity, bias, ease_to, ease_from;
        f = flg; if ((f & 1) && get(tension)) return -1;
        f >>= 1; if ((f & 1) && get(continuity)) return -1;
        f >>= 1; if ((f & 1) && get(bias)) return -1;
        f >>= 1; if ((f & 1) && get(ease_to)) return -1;
        f >>= 1; if ((f & 1) && get(ease_from)) return -1;
        if (get(arr, 4 * (n4 ? 4 : 3))) return -1;
        return 0;
    }
    int readTrack(const char *type, track_t &track, bool n4 = false) {
        i4_t d1, d2; u2_t flags; r4_t *arr = track.array;
        if (slot(flags) || get(d1) || get(d2) || get(track.size)) return -1;
        for (i4_t i = 0; i < track.size; i++) {
            if (readTCB(type, track.size, flags, arr, n4)) return -1;
            if (i < 100) arr += n4 ? 4 : 3;
        }
        return 0;
    }
    int readNode() {
        po_t e; if (end(e)) return -1;
        ds_node_t node; char name[64]; node.object[0] = name[0] = 0;
        u2_t child, id = (u2_t) -1, flags1 = 0, flags2 = 0, parent = (u2_t) -1;
        while (ok(e)) {
            switch (getChild(child)) {
            case 0xB010: if (readNodeHeader(node.object, flags1, flags2, parent)) return -1; continue;
            case 0xB011: if (readZStringSlot(name)) return -1; continue;
            case 0xB013: if (slot(node.pivot)) return -1; continue;
            case 0xB014: if (ignore("BOUNDBOX")) return -1; continue;
            case 0xB015: if (ignore("MORPH_SMOOTH")) return -1; continue;
            case 0xB020: if (readTrack("translate", node.translate)) return -1; continue;
            case 0xB021: if (readTrack("rotate", node.rotate, true)) return -1; continue;
            case 0xB022: if (readTrack("scale", node.scale)) return -1; continue;
            case 0xB023: if (ignore("FOV_TRACK_TAG")) return -1; continue;
            case 0xB024: if (ignore("ROLL_TRACK_TAG")) return -1; continue;
            case 0xB025: if (ignore("COL_TRACK_TAG")) return -1; continue;
            case 0xB026: if (ignore("MORPH_TRACK_TAG")) return -1; continue;
            case 0xB027: if (ignore("HOT_TRACK_TAG")) return -1; continue;
            case 0xB028: if (ignore("FALL_TRACK_TAG")) return -1; continue;
            case 0xB029: if (ignore("HIDE_TRACK_TAG")) return -1; continue;
            case 0xB030: if (slot(id)) return -1; continue;
            default: unknownChild(child); continue;
            }
        }
        node.id = ++id; node.parent = ++parent;
        if (m_reader) m_reader->nodeCB(&node);
        return 0;
    }
    int readKFData() {
        u2_t child; po_t e; if (end(e)) return -1;
        while (ok(e)) {
            switch (getChild(child)) {
            case 0x7001: if (ignore("VIEWPORT_LAYOUT")) return -1; continue;
            case 0xB001: if (ignore("AMBIENT_NODE_TAG")) return -1; continue;
            case 0xB002: if (readNode()) return -1; continue;
            case 0xB003: if (ignore("CAMERA_NODE_TAG")) return -1; continue;
            case 0xB004: if (ignore("TARGET_NODE_TAG")) return -1; continue;
            case 0xB005: if (ignore("LIGHT_NODE_TAG")) return -1; continue;
            case 0xB006: if (ignore("L_TARGET_NODE_TAG")) return -1; continue;
            case 0xB007: if (ignore("SPOTLIGHT_NODE_TAG")) return -1; continue;
            case 0xB008: if (ignore("KFSEG")) return -1; continue;
            case 0xB009: if (ignore("KFCURTIME")) return -1; continue;
            case 0xB00A: if (ignore("KFHDR")) return -1; continue;
            default: unknownChild(child); continue;
            }
        }
        return 0;
    }
    int readMain() {
        u2_t child; po_t e; if (end(e)) return -1;
        while (ok(e)) {
            switch (getChild(child)) {
            case 0x0002: if (readVersion()) return -1; continue;
            case 0x3D3D: if (readData()) return -1; continue;
            case 0xB000: if (readKFData()) return -1; continue;
            }
            return -1;
        }
        return 0;
    }
    int open3DS(const char *file, reader_t *reader = 0) {
        m_reader = reader;
        u2_t magic; m_fp = fopen(file, "rb");
        return m_fp && !get(magic) && magic == 0x4D4D ? 0 : -1;
    }
    io_t() : m_fp(0), m_reader(0) {}
    ~io_t() { if (m_fp) fclose(m_fp); }
};

inline int read3DS(const char *file, reader_t *reader)
{
    io_t in;
    return in.open3DS(file, reader) || in.readMain() ? -1 : 0;
}

/*######################################################################*/
