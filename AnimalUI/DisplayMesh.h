/*
Copyright (c) 2007 Ilya Baran

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef DISPLAYMESH_H
#define DISPLAYMESH_H

#include "../Libs/Pinocchio/mesh.h"

class DisplayMesh
{
public:
    virtual ~DisplayMesh() {}

    virtual const Mesh &getMesh() = 0;
    virtual vector<Vector3> getSkel() const { return vector<Vector3>(); }
};

class StaticDisplayMesh : public DisplayMesh
{
public:
    StaticDisplayMesh(const Mesh &inM) : m(inM) {}

    virtual const Mesh &getMesh() { return m; }
private:

    Mesh m;
};

#endif //DISPLAYMESH_H