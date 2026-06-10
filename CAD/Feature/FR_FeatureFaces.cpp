#include "./FR_FeatureFaces.h"

// Win includes
#ifdef _WIN32
#include <windows.h>
#endif

//-----------------------------------------------------------------------------

void FR::Dump(const FR_Feature& feature)
{
  std::wostringstream sstream;

  for ( FR_Feature::Iterator fit(feature); fit.More(); fit.Next() )
    sstream << fit.Key() << " ";

  sstream << std::endl;

  // Dump the debug (Windows-only) and standard outputs.
#ifdef _WIN32
  OutputDebugStringW( sstream.str().c_str() );
#endif
  std::cout << sstream.str().c_str();
}
