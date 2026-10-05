# pfm Format

Pfm (or Portable FloatMap) image format stores image as floating point pixels and hence are not restricted to usual 0-255 pixel value range. This is useful for HDR images or images that describes something other than colors like depth. 

One of the good viewer to view this file format is [PfmPad](https://sourceforge.net/projects/pfmpad/). We don't recommend Maverick photo viewer because it doesn't seem to show depth images properly.

HERCULES has code to write pfm file for [C++](https://github.com/lunarlab-gatech/HERCULES/blob/main/AirLib/include/common/common_utils/Utils.hpp#L697) and read as well as write for [Python](https://github.com/lunarlab-gatech/HERCULES/blob/main/PythonClient/hercules_cosysairsim/utils.py#L386).
