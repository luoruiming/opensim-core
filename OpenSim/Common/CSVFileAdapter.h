/* -------------------------------------------------------------------------- *
 *                          OpenSim:  CSVFileAdapter.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#ifndef OPENSIM_CSV_FILE_ADAPTER_H_
#define OPENSIM_CSV_FILE_ADAPTER_H_

#include "DelimFileAdapter.h"


namespace OpenSim {

/** CSVFileAdapter is a DelimFileAdapter that presets the delimiters 
appropriately for CSV files.                                                  */
class OSIMCOMMON_API CSVFileAdapter : public DelimFileAdapter {
public:
    CSVFileAdapter();
    CSVFileAdapter(const CSVFileAdapter&)            = default;
    CSVFileAdapter(CSVFileAdapter&&)                 = default;
    CSVFileAdapter& operator=(const CSVFileAdapter&) = default;
    CSVFileAdapter& operator=(CSVFileAdapter&&)      = default;
    ~CSVFileAdapter()                                = default;

    CSVFileAdapter* clone() const override;
};

}

#endif // OPENSIM_CSV_FILE_ADAPTER_H_
