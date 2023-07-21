#include <FL/Fl.H>
#include "MyWindow.h"
#include "DisplayMesh.h"
#include "processor.h"

int main(int argc, char **argv)
{
    MyWindow *window = new MyWindow();

    vector<string> args;
    for (int i = 0; i < argc; ++i)
        args.push_back(argv[i]);
    process(args, window);

    window->show();

    return Fl::run();
}
