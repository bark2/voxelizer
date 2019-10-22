// #include "stdafx.h"
#include "iritSkel.h"
#include "math.h"
#include "types.h"
#include <array>
#include <vector>

/*****************************************************************************
 * Skeleton for an interface to a parser to read IRIT data files.			 *
 ******************************************************************************
 * (C) Gershon Elber, Technion, Israel Institute of Technology                *
 ******************************************************************************
 * Written by:  Gershon Elber				Ver 1.0, Feb 2002 * Minimal changes made by Amit
 *Mano			November 2008					 *
 ******************************************************************************/

IPFreeformConvStateStruct CGSkelFFCState = { FALSE,          /* Talkative */
                                             FALSE,          /* DumpObjsAsPolylines */
                                             TRUE,           /* DrawFFGeom */
                                             FALSE,          /* DrawFFMesh */
                                             { 10, 10, 10 }, /* 10 isocurves peru/v/w direction. */
                                             100,            /* 100 point samples along a curve. */
                                             SYMB_CRV_APPROX_UNIFORM, /* CrvApproxMethod */
                                             FALSE,                   /* ShowIntrnal */
                                             FALSE,                   /* CubicCrvsAprox */
                                             20,                      /* Polygonal FineNess */
                                             FALSE,                   /* ComputeUV */
                                             TRUE,                    /* ComputeNrml */
                                             FALSE,                   /* FourPerFlat */
                                             0,                       /* OptimalPolygons */
                                             FALSE,                   /* BBoxGrid */
                                             TRUE,                    /* LinearOnePolyFlag */
                                             FALSE };

// CGSkelProcessIritDataFiles(argv + 1, argc - 1);

/*****************************************************************************
 * DESCRIPTION:                                                               *
 * Main module of skeleton - Read command line and do what is needed...	     *
 *                                                                            *
 * PARAMETERS:                                                                *
 *   FileNames:  Files to open and read, as a vector of strings.              *
 *   NumFiles:   Length of the FileNames vector.
 **
 *                                                                            *
 * RETURN VALUE:                                                              *
 *   bool:		false - fail, true - success.                                *
 *****************************************************************************/
bool
CGSkelProcessIritDataFiles(const char* file_name)
{
    IPObjectStruct* PObjects;
    IrtHmgnMatType  CrntViewMat;

    /* Get the data files: */
    IPSetFlattenObjects(FALSE);
    if ((PObjects = IPGetDataFiles(&file_name, 1 /*NumFiles*/, TRUE, FALSE)) == NULL) return false;
    PObjects = IPResolveInstances(PObjects);

    if (IPWasPrspMat)
        MatMultTwo4by4(CrntViewMat, IPViewMat, IPPrspMat);
    else
        IRIT_GEN_COPY(CrntViewMat, IPViewMat, sizeof(IrtHmgnMatType));

    /* Here some useful parameters to play with in tesselating freeforms: */
    CGSkelFFCState.FineNess          = 20;   /* Res. of tesselation, larger is finer. */
    CGSkelFFCState.FourPerFlat       = TRUE; /* 4 poly per ~flat patch, 2 otherwise.*/
    CGSkelFFCState.LinearOnePolyFlag = TRUE; /* Linear srf gen. one poly. */

    /* Traverse ALL the parsed data, recursively. */
    IPTraverseObjListHierarchy(PObjects, CrntViewMat, CGSkelDumpOneTraversedObject);
    return true;
}

/*****************************************************************************
 * DESCRIPTION:                                                               *
 *   Call back function of IPTraverseObjListHierarchy. Called on every non    *
 * list object found in hierarchy.                                            *
 *                                                                            *
 * PARAMETERS:                                                                *
 *   PObj:       Non list object to handle.                                   *
 *   Mat:        Transformation matrix to apply to this object.               *
 *   Data:       Additional data.                                             *
 *                                                                            *
 * RETURN VALUE:                                                              *
 *   void *
 *****************************************************************************/
void
CGSkelDumpOneTraversedObject(IPObjectStruct* PObj, IrtHmgnMatType Mat, void* Data)
{
    IPObjectStruct* PObjs;

    if (IP_IS_FFGEOM_OBJ(PObj))
        PObjs = IPConvertFreeForm(PObj, &CGSkelFFCState);
    else
        PObjs = PObj;

    for (PObj = PObjs; PObj != NULL; PObj = PObj->Pnext)
        if (!CGSkelStoreData(PObj)) exit(1);
}

/*****************************************************************************
 * DESCRIPTION:                                                               *
 *   Prints the data from given geometry object.
 **
 *                                                                            *
 * PARAMETERS:                                                                *
 *   PObj:       Object to print.                                             *
 *   Indent:     Column of indentation.                                       *
 *                                                                            *
 * RETURN VALUE:                                                              *
 *   bool:		false - fail, true - success.                                *
 *****************************************************************************/

void
triangulate_ear_clipping(IPPolygonStruct* poly)
{
    using Triangle = std::array<std::array<double, 3>, 3>;
    using IVoxelizer::epsilon;
    using IVoxelizer::is_point_in_triangle;
    using IVoxelizer::vec2;
    using IVoxelizer::vec3;

    extern double                 scene_aabb_min[3];
    extern double                 scene_aabb_max[3];
    extern std::vector<Triangle>  triangles;
    extern std::vector<Triangle*> meshes;
    extern std::vector<size_t>    meshes_size;

    IPVertexStruct* p;
    IPVertexStruct* prev;
    IPVertexStruct* curr;
    IPVertexStruct* next;
    IPVertexStruct* botleft;
    Triangle        t;

    vec3   n       = { poly->Plane[0], poly->Plane[1], poly->Plane[2] };
    double inv     = 1 / n.length();
    double cos     = n[2] * inv;
    double sin     = std::sqrt(std::pow(n[0], 2) + std::pow(n[1], 2)) * inv;
    double u1      = n[1] * inv;
    double u2      = n[0] * inv;
    auto   proj_xy = [=](IPVertexStruct* pv) {
        vec3 v  = { pv->Coord[0], pv->Coord[1], pv->Coord[2] };
        vec3 m1 = { cos + std::pow(u1, 2) * (1 - cos), u1 * u2 * (1 - cos), u2 * sin };
        vec3 m2 = { u1 * u2 * (1 - cos), cos + std::pow(u2, 2) * (1 - cos), -u1 * sin };
        return vec2(dot(m1, v), dot(m2, v));
    };

    int vertex_count = 1;
    for (prev = poly->PVertex; prev->Pnext; prev = prev->Pnext, vertex_count++)
        ;
    prev->Pnext = poly->PVertex;
    printf("vertex count: %d\n", vertex_count);

    prev    = poly->PVertex;
    botleft = prev;
    for (int i = 0; i < vertex_count; i++) {
        if (proj_xy(prev).x < proj_xy(botleft).x ||
            (proj_xy(prev).x == proj_xy(botleft).x && proj_xy(prev).y < proj_xy(botleft).y))
            botleft = prev;
        prev = prev->Pnext;
    }

    for (prev = poly->PVertex; prev->Pnext != botleft;) prev = prev->Pnext;
    next        = botleft->Pnext;
    bool is_ccw = IVoxelizer::signed_edge_function(proj_xy(prev), proj_xy(botleft), proj_xy(next)) < 0.0;

    prev         = poly->PVertex;
    curr         = prev->Pnext;
    next         = curr->Pnext;
    int prev_idx = 0, curr_idx = 1, next_idx = 2, p_idx;

    for (; vertex_count >= 3;) {
        printf("prev: %d, curr: %d, next: %d\n", prev_idx, curr_idx, next_idx);
        // assumes the polygon is formed in a counter clockwise fashion
        vec2 proj_prev = proj_xy(prev);
        vec2 proj_curr = proj_xy(curr);
        vec2 proj_next = proj_xy(next);

        std::array<vec2, 3> proj_tri = { proj_prev, proj_curr, proj_next };
        bool concave = is_ccw ? IVoxelizer::signed_edge_function(proj_prev, proj_next, proj_curr) < 0.0
                              : IVoxelizer::signed_edge_function(proj_prev, proj_next, proj_curr) > 0.0;

        if (concave) {
            prev     = prev->Pnext;
            curr     = curr->Pnext;
            next     = next->Pnext;
            prev_idx = (prev_idx + 1 == vertex_count) ? 0 : prev_idx + 1;
            curr_idx = (curr_idx + 1 == vertex_count) ? 0 : curr_idx + 1;
            next_idx = (next_idx + 1 == vertex_count) ? 0 : next_idx + 1;
            if (prev == poly->PVertex) {
                printf("ear not found\n");
                break;
            }
            continue;
        }

        bool is_ear = true;
        p           = poly->PVertex;
        int p_idx   = 0;
        do {
            printf("\ttest p: %d\n", p_idx);
            vec2 proj_p = proj_xy(p);

            if (p == prev || p == curr || p == next) {
                p_idx++;
                continue;
            }

            if (is_point_in_triangle(proj_tri, true, proj_p)) {
                is_ear = false;
                break;
            }

            p = p->Pnext;
            p_idx++;
        } while (p_idx != vertex_count);

        if (!is_ear) {
            prev     = prev->Pnext;
            curr     = curr->Pnext;
            next     = next->Pnext;
            prev_idx = (prev_idx + 1 == vertex_count) ? 0 : prev_idx + 1;
            curr_idx = (curr_idx + 1 == vertex_count) ? 0 : curr_idx + 1;
            next_idx = (next_idx + 1 == vertex_count) ? 0 : next_idx + 1;
            if (prev == poly->PVertex) {
                printf("ear not found\n");
                break;
            }
            continue;
        }

        t[0] = { prev->Coord[0], prev->Coord[1], prev->Coord[2] };
        t[1] = { curr->Coord[0], curr->Coord[1], curr->Coord[2] };
        t[2] = { next->Coord[0], next->Coord[1], next->Coord[2] };
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                scene_aabb_max[j] = std::max(scene_aabb_max[j], t[i][j]);
                scene_aabb_min[j] = std::min(scene_aabb_min[j], t[i][j]);
            }
        }

        triangles.push_back(t);
        prev->Pnext = next; // FIXME: memory leak
        curr        = next;
        next        = next->Pnext;
        vertex_count--;
        printf("ear found\n");
    }
}

void
triangulate_convex(IPPolygonStruct* poly)
{
    using Triangle = std::array<std::array<double, 3>, 3>;

    extern double                scene_aabb_min[3];
    extern double                scene_aabb_max[3];
    extern std::vector<Triangle> triangles;
    IPVertexStruct*              PVertex;

    Triangle t;
    int      vi = 0;
    PVertex     = poly->PVertex;
    do {
        for (int j = 0; j < 3; j++) {
            double x          = PVertex->Coord[j];
            scene_aabb_max[j] = std::max(scene_aabb_max[j], x);
            scene_aabb_min[j] = std::min(scene_aabb_min[j], x);
            if (vi < 3)
                t[vi][j] = x;
            else {
                t[1][j] = t[2][j];
                t[2][j] = x;
            }
        }

        vi++;
        if (vi >= 3) triangles.push_back(t);

        PVertex = PVertex->Pnext;
    } while (PVertex != poly->PVertex && PVertex != NULL);
}

bool
CGSkelStoreData(IPObjectStruct* PObj)
{
    if (PObj->ObjType != IP_OBJ_POLY) return true;

    for (auto poly = PObj->U.Pl; poly != NULL; poly = poly->Pnext) {
        if (poly->PVertex == NULL) return false;
        // triangulate_earclipping(poly);
        triangulate_convex(poly);
    }

    return true;
}

/*****************************************************************************
 * DESCRIPTION:                                                               *
 *   Returns the color of an object.                                          *
 *                                                                            *
 * PARAMETERS:                                                                *
 *   PObj:   Object to get its color.                                         *
 *   RGB:    as 3 floats in the domain [0, 1].                                *
 *                                                                            *
 * RETURN VALUE:                                                              *
 *   int:    TRUE if object has color, FALSE otherwise.                       *
 *****************************************************************************/
int
CGSkelGetObjectColor(IPObjectStruct* PObj, double RGB[3])
{
    static int TransColorTable[][4] = {
        { /* BLACK	*/ 0, 0, 0, 0 },        { /* BLUE	*/ 1, 0, 0, 255 },
        { /* GREEN	*/ 2, 0, 255, 0 },      { /* CYAN	*/ 3, 0, 255, 255 },
        { /* RED	*/ 4, 255, 0, 0 },      { /* MAGENTA 	*/ 5, 255, 0, 255 },
        { /* BROWN	*/ 6, 50, 0, 0 },       { /* LIGHTGRAY	*/ 7, 127, 127, 127 },
        { /* DARKGRAY	*/ 8, 63, 63, 63 },     { /* LIGHTBLUE	*/ 9, 0, 0, 255 },
        { /* LIGHTGREEN	*/ 10, 0, 255, 0 },     { /* LIGHTCYAN	*/ 11, 0, 255, 255 },
        { /* LIGHTRED	*/ 12, 255, 0, 0 },     { /* LIGHTMAGENTA */ 13, 255, 0, 255 },
        { /* YELLOW	*/ 14, 255, 255, 0 },   { /* WHITE	*/ 15, 255, 255, 255 },
        { /* BROWN	*/ 20, 50, 0, 0 },      { /* DARKGRAY	*/ 56, 63, 63, 63 },
        { /* LIGHTBLUE	*/ 57, 0, 0, 255 },     { /* LIGHTGREEN	*/ 58, 0, 255, 0 },
        { /* LIGHTCYAN	*/ 59, 0, 255, 255 },   { /* LIGHTRED	*/ 60, 255, 0, 0 },
        { /* LIGHTMAGENTA */ 61, 255, 0, 255 }, { /* YELLOW	*/ 62, 255, 255, 0 },
        { /* WHITE	*/ 63, 255, 255, 255 }, { -1, 0, 0, 0 }
    };
    int i, j, Color, RGBIColor[3];

    if (AttrGetObjectRGBColor(PObj, &RGBIColor[0], &RGBIColor[1], &RGBIColor[2])) {
        for (i = 0; i < 3; i++) RGB[i] = RGBIColor[i] / 255.0;

        return TRUE;
    }
    else if ((Color = AttrGetObjectColor(PObj)) != IP_ATTR_NO_COLOR) {
        for (i = 0; TransColorTable[i][0] >= 0; i++) {
            if (TransColorTable[i][0] == Color) {
                for (j = 0; j < 3; j++) RGB[j] = TransColorTable[i][j + 1] / 255.0;
                return TRUE;
            }
        }
    }

    return FALSE;
}

/*****************************************************************************
 * DESCRIPTION:                                                               *
 *   Returns the volumetric texture of an object, if any.                     *
 *                                                                            *
 * PARAMETERS:                                                                *
 *   PObj:   Object to get its volumetric texture.                            *
 *                                                                            *
 * RETURN VALUE:                                                              *
 *   char *:    Name of volumetric texture map to apply, NULL if none.        *
 *****************************************************************************/
const char*
CGSkelGetObjectTexture(IPObjectStruct* PObj)
{
    return AttrGetObjectStrAttrib(PObj, "texture");
}

/*****************************************************************************
 * DESCRIPTION:                                                               *
 *   Returns the parametric texture of an object, if any.                     *
 *                                                                            *
 * PARAMETERS:                                                                *
 *   PObj:   Object to get its parametric texture.                            *
 *                                                                            *
 * RETURN VALUE:                                                              *
 *   char *:    Name of parametric texture map to apply, NULL if none.        *
 *****************************************************************************/
const char*
CGSkelGetObjectPTexture(IPObjectStruct* PObj)
{
    return AttrGetObjectStrAttrib(PObj, "ptexture");
}

/*****************************************************************************
 * DESCRIPTION:                                                               *
 *   Returns the transparency level of an object, if any.                     *
 *                                                                            *
 * PARAMETERS:                                                                *
 *   PObj:   Object to get its volumetric texture.                            *
 *   Transp: Transparency level between zero and one.                         *
 *                                                                            *
 * RETURN VALUE:                                                              *
 *   int:    TRUE if object has transparency, FALSE otherwise.                *
 *****************************************************************************/
int
CGSkelGetObjectTransp(IPObjectStruct* PObj, double* Transp)
{
    *Transp = AttrGetObjectRealAttrib(PObj, "transp");

    return !IP_ATTR_IS_BAD_REAL(*Transp);
}
