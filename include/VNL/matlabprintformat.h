#ifndef vnl_matlab_print_format_h_
#define vnl_matlab_print_format_h_
/*
  fsm@robots.ox.ac.uk
*/

/**
* \file
*/

namespace VNL {

/** pretty-printing matlab formats.
*/
enum MatlabPrintFormat {
  matlab_print_format_default,
  matlab_print_format_short,
  matlab_print_format_long,
  matlab_print_format_short_e,
  matlab_print_format_long_e
};

// -------------------- Setting the default format.

/** get top of stack :.
*/
MatlabPrintFormat MatlabPrintFormatTop();

/** set new, get old format at top of stack :.
*/
MatlabPrintFormat MatlabPrintFormatSet(MatlabPrintFormat);

/** push/pop the top of the stack :.
*/
void MatlabPrintFormatPush(MatlabPrintFormat);
void MatlabPrintFormatPop ();

}; // End namespace VNL

#endif
