#include "projectiveConvexQuickHull.h"
#include "mesh3d.h"

using namespace std;

namespace corecvs {

double pointLineDist(const ProjectiveCoord4d &lineP1, const ProjectiveCoord4d &lineP2, const ProjectiveCoord4d &point) {
    if (lineP1 == point || lineP2 == point)
        return 0;
    if (lineP1.w() == 0 || lineP2.w() == 0 || point.w() == 0)
        return std::numeric_limits<double>::max();
    ProjectiveCoord4d lineVect = lineP2 - lineP1;
    lineVect = lineVect + ProjectiveCoord4d(0, 0, 0, 1);
    return (lineVect.toVector() ^ (point.toVector() - lineP1.toVector())).l2Metric() / lineVect.xyz().l2Metric();
}

double pointPlaneDist(const ProjectiveCoord4d &planeP1, const ProjectiveCoord4d &planeP2,
                      const ProjectiveCoord4d &planeP3, const ProjectiveCoord4d &point) {
    if (planeP1 == point || planeP2 == point || planeP3 == point)
        return 0;
    if (planeP1.w() == 0 || planeP2.w() == 0 || planeP3.w() == 0 || point.w() == 0)
        return std::numeric_limits<double>::infinity();
    auto baseV1 = planeP2.toVector() - planeP1.toVector();
    auto baseV2 = planeP3.toVector() - planeP1.toVector();
    auto prod = baseV1 & (baseV2 ^ (point.toVector() - planeP1.toVector()));
    return prod / ((baseV1 ^ baseV2).l2Metric());
}

double pointFaceDist(const Triangle4dd &face, const ProjectiveCoord4d &point) {
    return pointPlaneDist(face.p1(), face.p2(), face.p3(), point);
}

bool faceIsVisible(const ProjectiveCoord4d &eyePoint, const ProjectiveConvexQuickHull::HullFace &face, double eps) {
    cout << "eye: " << eyePoint << "; " << face.plane.p1() << " " << face.plane.p2() << " " << face.plane.p3() << "\n";

    if (eyePoint == face.plane.p1() || eyePoint == face.plane.p2() || eyePoint == face.plane.p3())
        return false;
    // one line
    if (face.plane.p1() == face.plane.p2() || face.plane.p1() == face.plane.p3() || face.plane.p1() == face.plane.p3())
        return false;

    // (1, 1, 1, 0)
    if (face.is_Inf() || face.is_Low()) {
        cout << "(1,1,1,0)\n";
        Vector3dd p = (face.plane.p1() - face.plane.p2()).xyz();
        Vector3dd q = (face.plane.p1() - face.plane.p3()).xyz();
        auto normal = p ^ q;
        cout << "cute normal: " << normal << "\n";
        auto res = -1 * face.plane.p1().xyz() & normal.normalised();
        return res >= eps;
    }
    // ox | oy | oz plane
    if ((face.plane.p1().x() == 0 && face.plane.p2().x() == 0 && face.plane.p3().x() == 0) ||
        (face.plane.p1().y() == 0 && face.plane.p2().y() == 0 && face.plane.p3().y() == 0) ||
        (face.plane.p1().z() == 0 && face.plane.p2().z() == 0 && face.plane.p3().z() == 0)) {
        Vector3dd p = (face.plane.p1() - face.plane.p2()).xyz();
        Vector3dd q = (face.plane.p1() - face.plane.p3()).xyz();
        auto normal = p ^ q;
        auto res = -1 * face.plane.p1().xyz() & normal.normalised();
        cout << "cute normal: " << normal << ": " << res << "\n";
        return res >= eps;
    }
    // 2 points on infinity
    if ((face.plane.p1().w() == 0 && face.plane.p2().w() == 0) ||
        (face.plane.p1().w() == 0 && face.plane.p3().w() == 0) ||
        (face.plane.p2().w() == 0 && face.plane.p3().w() == 0)) {
        cout << "2 points on infinity\n";
        Vector3dd p = (face.plane.p1() - face.plane.p2()).xyz();
        Vector3dd q = (face.plane.p1() - face.plane.p3()).xyz();
        auto normal = p ^ q;
        auto res = -1 * face.plane.p1().xyz() & normal.normalised();
        cout << "cute normal: " << normal << ": " << res << "\n";
        return res >= eps;
    }

    Vector3dd normal = face.plane.getNormal();
    Vector3dd point = face.plane.p1().toVector();
    // (1, 0, 0, 0)
    if (face.plane.p1().w() == 0) {
        Vector3dd q = face.plane.p1().xyz();
        ProjectiveCoord4d p = face.plane.p2() - face.plane.p3();
        normal = q ^ p.toVector();
        point = face.plane.p2().toVector();
    } else if (face.plane.p2().w() == 0) {
        Vector3dd q = face.plane.p2().xyz();
        ProjectiveCoord4d p = face.plane.p1() - face.plane.p3();
        normal = q ^ p.toVector();
    } else if (face.plane.p3().w() == 0) {
        Vector3dd q = face.plane.p3().xyz();
        ProjectiveCoord4d p = face.plane.p1() - face.plane.p2();
        normal = p.toVector() ^ q;
    }
    cout << normal << " " << point << "\n";
    if (eyePoint.w() == 0) {
        double t = eyePoint.xyz() & normal;
        cout << "inf eye: " << t << "\n";
        return t >= eps;
    }

    auto res = ((eyePoint.toVector() - point) & normal.normalised());
    cout << normal << " " << res << "\n\n";
    return res >= eps;
}

void addPointsToFaces(ProjectiveConvexQuickHull::HullFace *faces, unsigned long faces_count,
        const ProjectiveConvexQuickHull::Vertices &listVertices, double eps) {
    for (const ProjectiveCoord4d &vertex : listVertices)
        for (unsigned long i = 0; i < faces_count; i++) {
            auto &face = faces[i];
            if (faceIsVisible(vertex, face, eps)) {
                faces[i].points.push_back(vertex);
                break;
            }
        }
}

ProjectiveConvexQuickHull::Vertices createSimplex(const ProjectiveConvexQuickHull::Vertices& listVertices) {
    ProjectiveCoord4d first = listVertices.front();
    ProjectiveCoord4d EP[6] = {first, first, first, first, first, first};

    for (const auto &vertex : listVertices) {
        if (vertex.X() <= EP[0].X()) EP[0] = vertex;
        if (vertex.X() >= EP[1].X()) EP[1] = vertex;
        if (vertex.Y() <= EP[2].Y()) EP[2] = vertex;
        if (vertex.Y() >= EP[3].Y()) EP[3] = vertex;
        if (vertex.Z() <= EP[4].Z()) EP[4] = vertex;
        if (vertex.Z() >= EP[5].Z()) EP[5] = vertex;
    }
    cout << "EP:";
    for (size_t t = 0; t < 6; t++ )
        cout << t << " " << EP[t] << endl;

    ProjectiveCoord4d triangleP1 = EP[0], triangleP2 = EP[0], triangleP3 = EP[0];
    double maxDist = -1;
    for (const ProjectiveCoord4d &point1 : EP)
        for (const ProjectiveCoord4d &point2 : EP) {
            double dist = point2.dist(point1);
            if (dist >= maxDist && point1 != point2) {
                maxDist = dist;
                triangleP1 = point1;
                triangleP2 = point2;
            }
        }

    maxDist = pointLineDist(triangleP1, triangleP2, triangleP3);
    for (const auto &point : EP) {
        double dist = pointLineDist(triangleP1, triangleP2, point);
        if (dist >= maxDist && point != triangleP1 && point != triangleP2) {
            maxDist = dist;
            triangleP3 = point;
        }
    }
    cout << "3 triangles: \n" << triangleP1 << "\n" << triangleP2 << "\n" << triangleP3 << "\n";

    maxDist = 0;
    ProjectiveCoord4d apex = EP[0];
    for (const auto &point : listVertices) {
        cout << point << "\n";
        double dist = abs(pointPlaneDist(triangleP1, triangleP2, triangleP3, point));
        if (dist == 0)
            continue;
        bool f = faceIsVisible(triangleP3, ProjectiveConvexQuickHull::HullFace(apex, triangleP1, triangleP2), 0);
        bool f1 = faceIsVisible(point, ProjectiveConvexQuickHull::HullFace(apex, triangleP1, triangleP2), 0);
        bool f2 = faceIsVisible(point, ProjectiveConvexQuickHull::HullFace(apex, triangleP2, triangleP1), 0);
        bool res = f ? f1 : f2;
        cout << "count face:" << point << ": " << f << ", " << f1 << ", " << f2 << "\n";
        if (dist >= maxDist && point != triangleP1 && point != triangleP2 && point != triangleP3 && !res) {
            maxDist = dist;
            apex = point;
        }
    }
    cout << "apex\n" << apex << "\n";
    ProjectiveConvexQuickHull::Vertices Res;
    if (faceIsVisible(apex, ProjectiveConvexQuickHull::HullFace(triangleP1, triangleP2, triangleP3), 0))
        Res = { triangleP1, triangleP3, triangleP2, apex };
    else Res = { triangleP1, triangleP2, triangleP3, apex };
    return Res;
}

ProjectiveConvexQuickHull::HullFaces ProjectiveConvexQuickHull::quickHull(const ProjectiveConvexQuickHull::Vertices &listVertices, double epsilon) {
    if (listVertices.size() < 4)
        return ProjectiveConvexQuickHull::HullFaces();

    Vertices simplex = createSimplex(listVertices);
    cout << "simplex:";
    for (size_t t = 0; t < simplex.size(); t++ )
        cout << t << " " << simplex[t] << endl;
    Vertices uniqueSimplex;

    for (ProjectiveCoord4d& elem : simplex)
        if (find(uniqueSimplex.begin(), uniqueSimplex.end(), elem) == uniqueSimplex.end())
            uniqueSimplex.push_back(elem);

    if (uniqueSimplex.size() == 1) {
        printf("Only one unique point\n");
        return {};
    }

    if (uniqueSimplex.size() == 2) {
        printf("This is line\n");
        return {};
    }

    if (uniqueSimplex.size() == 3) {
        printf("This is plane\n");
        return {};
    }
    cout << "unique simplex:";
    for (size_t t = 0; t < uniqueSimplex.size(); t++ )
        cout << t << " " << uniqueSimplex[t] << endl;

    HullFaces faces = {
            HullFace(simplex[0], simplex[1], simplex[2]),
            HullFace(simplex[0], simplex[2], simplex[3]),
            HullFace(simplex[1], simplex[3], simplex[2]),
            HullFace(simplex[0], simplex[3], simplex[1])};

    queue<HullFace> queue;
    addPointsToFaces(faces.data(), faces.size(), listVertices, epsilon);
    for (auto face : faces) {
        cout << "face: " << face.plane.p1() << " " << face.plane.p2() << " " << face.plane.p3() << "\n";
        cout << "points: ";
        for (int i = 0; i < face.points.size(); i++)
            cout << face.points[i] << " ";
        cout << "\n\n";
        if (!face.points.empty())
            queue.push(face);
    }

    while (!queue.empty()) {
        HullFace face = queue.front();
        queue.pop();

        if (!face.points.empty()) {
            double maxDist = -1;
            ProjectiveCoord4d furthest = {0, 0, 0, 1};
            for (const auto &point : face.points) {
                double dist = pointFaceDist(face.plane, point);
                if (dist > maxDist) {
                    maxDist = dist;
                    furthest = point;
                }
            }

            Vertices listUnclaimedVertices;
            HullFaces newFaces;

            for (HullFace &curFace : faces)
                if (faceIsVisible(furthest, curFace, epsilon)) {
                    listUnclaimedVertices.insert(listUnclaimedVertices.end(), curFace.points.begin(), curFace.points.end());

                    HullFaces tmpFaces;
                    tmpFaces.reserve(3);
                    tmpFaces.emplace_back(curFace.plane.p3(), furthest, curFace.plane.p2());
                    tmpFaces.emplace_back(curFace.plane.p1(), curFace.plane.p2(), furthest);
                    tmpFaces.emplace_back(curFace.plane.p1(), furthest, curFace.plane.p3());


                    for (HullFace &tmpFace : tmpFaces) {
                        auto alreadyExists = find(newFaces.begin(), newFaces.end(), tmpFace);
                        if (alreadyExists != newFaces.end())
                            newFaces.erase(alreadyExists);
                        else
                            newFaces.push_back(tmpFace);
                    }
                    curFace.toDelete = true;
                }

            faces.erase(remove_if(
                    faces.begin(),
                    faces.end(),
                    [](HullFace &face) {return face.toDelete;}
            ), faces.end());

            addPointsToFaces(newFaces.data(), newFaces.size(), listUnclaimedVertices, epsilon);
            for (auto &newFace : newFaces) {
                faces.push_back(newFace);
                queue.push(newFace);
            }
        }
    }
    cout << "Right result: \n";
    for (auto &face: faces)
        cout << face.plane.p1() << " " << face.plane.p2() << " " << face.plane.p3() << "\n";
    return faces;
}

} // namespace corecvs
