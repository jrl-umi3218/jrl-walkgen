enable_language (Fortran)

SET(SOURCES_FORTRAN
  dgesvd.f
  dgesvx.f
  dgetrf.f
  gdgges.f
  dlamch.f
  dlapy2.f
  dgetrf2.f
  gdtgsen.f
  gdtgexc.f
  gdtgex2.f
  gdgemm.f
  gsgemm.f
)

ADD_LIBRARY(sublapack SHARED ${SOURCES_FORTRAN})

