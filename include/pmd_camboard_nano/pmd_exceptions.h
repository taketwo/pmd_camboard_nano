/******************************************************************************
 * Copyright (c) 2012 Sergey Alexandrov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/

#ifndef PMD_EXCEPTIONS_H
#define PMD_EXCEPTIONS_H

#include <stdexcept>

namespace pmd_camboard_nano
{

class PMDException : public std::runtime_error
{

public:

  PMDException(int status, const std::string& msg = "")
  : std::runtime_error(msg)
  , status_code(status_code)
  { };

  int status_code;

};

class PMDPluginNotFoundException : public PMDException
{

public:

  PMDPluginNotFoundException(const std::string& msg = "")
  : PMDException(PMD_FILE_NOT_FOUND, msg)
  { }

};

class PMDCameraNotOpenedException : public PMDException
{

public:

  PMDCameraNotOpenedException(const std::string& msg = "")
  : PMDException(PMD_COULD_NOT_OPEN, msg)
  { }

};

}

#endif /* PMD_EXCEPTIONS_H */

