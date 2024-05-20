#ifndef DMPLIB_UTILS_MACROS_HPP__
#define DMPLIB_UTILS_MACROS_HPP__

#define GET(var, desc)                                                                 \
    inline decltype(var) get_##desc() const { return var; }

#define SET(var, desc)                                                                 \
    inline void set_##desc(const decltype(var)& value) { var = value; }

#define GET_SET(var, desc)                                                             \
    GET(var, desc)                                                                     \
    SET(var, desc)

#define GETPTR(var, desc)                                                              \
    inline decltype(var)* get_##desc() { return &var; }

#define GETCONSTPTR(var, desc)                                                         \
    inline const decltype(var)* get_##desc() const { return &var; }


#endif  // DMPLIB_UTILS_MACROS_HPP__
