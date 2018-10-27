within MbsLite.Util;

package Constants

  constant Real[3] vertical  = { 0, 1, 0 };
  constant Real[3] forward   = { 1, 0, 0 };
  constant Real[3] userward  = { 0, 0, 1 };

  constant Real[3] gravity   = -vertical;

  constant Integer FIXME_N_WHEELS   = 3 "due to a nasty bug in OpenModelica, specifying records of initially unknown length or of one specified by a parameter, especially when all of this is happening while using records, the compiler is just like '........ Meh.' :(";

end Constants;

