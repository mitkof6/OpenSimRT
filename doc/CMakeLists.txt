################################################################################
# build doxygen

# add a target to generate API documentation with Doxygen
find_package(Doxygen)

if(BUILD_DOCUMENTATION)
  if(NOT DOXYGEN_FOUND)
    message(FATAL_ERROR "Doxygen is needed to build the documentation.")
  endif()

  # Try to use MathJax for Doxygen equations.
  option(DOXYGEN_USE_MATHJAX "Try to use MathJax to render Doxygen
        equations. If OFF, LATEX is used instead if you have it; otherwise we use
        MathJax from the internet (we don't download it). If we cannot download MathJax
        from the internet, we turn this option OFF." OFF)

  set(MATHJAX_ZIP_PATH "${PROJECT_BINARY_DIR}/MathJax.zip")

  ## LaTeX or MathJax
  # -----------------
  # Doxygen must render equations, and this can be done either with LaTeX or
  # MathJax. LaTeX is unpleasant to install on Windows, so we prefer MathJax.

  # There are two ways of using MathJax: from the internet, or locally.
  # We prefer using MathJax locally, but we have to download it first.
  # We download a 50 mb MathJax zip and unzip it (150 mb).
  # The zip can be thrown away, but the 150 mb MathJax directory must stay
  # with the Doxygen documentation.

  # If we are unable to download MathJax, and the user has LaTeX, we fall
  # back onto LaTeX. If the user has neither MathJax *or* LaTeX, we fall back
  # on using MathJax from the internet instead of downloading it.

  # This block of commands is all executing during the configuring phase.
  if(${DOXYGEN_USE_MATHJAX} AND NOT EXISTS "${MATHJAX_ZIP_PATH}")
    message(STATUS "Trying to download MathJax for Doxygen...")
    file(DOWNLOAD https://github.com/mathjax/MathJax/archive/v2.5-latest.zip
      "${MATHJAX_ZIP_PATH}"
      #SHOW_PROGRESS Clutters the CMake output.
      STATUS MATHJAX_DOWNLOAD_STATUS
      )

    # Did we download MathJax successfully?
    # Get first element of the status.
    list(GET MATHJAX_DOWNLOAD_STATUS 0 MATHJAX_DOWNLOAD_RETURN)
    if(${MATHJAX_DOWNLOAD_RETURN} EQUAL 0)
      message(STATUS "Successfully downloaded MathJax.")
    else()
      # Must modify the cache variable so we don't end up here after
      # running the CMake configuring step multiple times.
      set(DOXYGEN_USE_MATHJAX OFF CACHE BOOL
        "Overridden to OFF since we could not download MathJax."
        FORCE)
      list(GET MATHJAX_DOWNLOAD_STATUS 1 MATHJAX_DOWNLOAD_MESSAGE)
      message(STATUS
        "Could not download MathJax: ${MATHJAX_DOWNLOAD_MESSAGE}")
    endif()
  endif()

  if(${DOXYGEN_USE_MATHJAX})
    # This means the user wants to use MathJax *and* we were able to
    # download it successfully.

    # See Doxyfile.in for a description of these variables.
    set(DOXYGEN_USE_MATHJAX_DOXYFILE "YES")
    set(DOXYGEN_MATHJAX_RELPATH "MathJax-2.5-latest")

  else()
    # Try using LATEX.
    find_package(LATEX)
    if(${LATEX_FOUND})
      set(DOXYGEN_USE_MATHJAX_DOXYFILE "NO")
      message(STATUS "Using LATEX for Doxygen equations.")
    else()
      message(STATUS
        "Using MathJax Content Distribution Network for Doxygen.")
      set(DOXYGEN_USE_MATHJAX_DOXYFILE "YES")
      set(DOXYGEN_MATHJAX_RELPATH
        "https://cdn.mathjax.org/mathjax/latest")
    endif()
  endif()


  set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
  set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  configure_file(${doxyfile_in} ${doxyfile} @ONLY)

  add_custom_target(Doxygen ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM)

  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
    DESTINATION "${CMAKE_INSTALL_DOCDIR}")
endif()
