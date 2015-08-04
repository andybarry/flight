


                              SMC -
                     The State Machine Compiler


+ Building with statemap.h and statemap.m
-----------------------------------------

SMC does not build a statemap library. The Objective C
developer must either:

    1. Build <SMC directory>/lib/ObjC/statemap.m along with
       the application .m source files or

    2. Build statemap.m into an existing library or create
       a new library and then link that library into the
       application.

The reason SMC does not provide an prebuilt library is due
to the complexity and effort needed to maintain the
configuration files for all compiler, operating system pairs
for one small .m file.

(Aside: that is why the SMC uses only a header file for C and
        C++ - no linking, just a #include.)
