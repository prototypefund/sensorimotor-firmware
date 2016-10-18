#ifndef SUPREME_TOOLS_MODULES_H
#define SUPREME_TOOLS_MODULES_H

namespace supreme { namespace tools {

inline int sgn(float x) {
    if (x > 0) return +1;
    else if (x < 0) return -1;
    else return 0;
}

}} // namespace supreme::tools

#endif // SUPREME_TOOLS_MODULES_H
