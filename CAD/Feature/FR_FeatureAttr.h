
#ifndef FR_FeatureAttr_h
#define FR_FeatureAttr_h

// FR includes
//#include <FR_JsonDict.h>
#include "./FR_Utils.h"

// Active Data includes
//#include <ActAPI_IPlotter.h>

#include <assert.h>

// OCCT includes
#include <Standard_GUID.hxx>
//#include <CskTCollection_AsciiString.hxx>

class FR_AAG;

//-----------------------------------------------------------------------------

//! \ingroup ASI_AFR
//!
//! Base class for all feature attributes.
class FR_FeatureAttr : public Standard_Transient
{
friend class FR_AAG;

  // OCCT RTTI
  DEFINE_STANDARD_RTTI_INLINE(FR_FeatureAttr, Standard_Transient)

public:

  virtual ~FR_FeatureAttr() {}

public:

  virtual const Standard_GUID&
    GetGUID() const = 0;

  virtual const char*
    GetName() const = 0;

public:

  /** @name Serialization
   *  Methods to serialize and deserialize the AAG attribute.
   */
  //@{

  //! Serializes this attribute to the binary format. The file handler `pFile`
  //! should be opened and prepared for writing.
  //! \param[in] pFile the C file handler.
  //! \return false if writing is impossible or not done.
  virtual bool
    Serialize(FILE* pFile) const
  {
    (void) pFile;
    return false;
  }

  //@}

public:

  //! Dumps the attribute into a single line (to be used in titles).
  //! \return brief one-line dump of the attribute.
  //virtual TCollection_AsciiString DumpInline() const { return ""; }

  //! Dumps the attribute to the passed output stream.
  //! \param[out] out target output stream.
  virtual void Dump(std::ostream& out) const
  {}

  //! Dumps this attribute as a JSON object.
  //! \param[in,out] out       target output stream.
  //! \param[in]     numSpaces number of spaces to use for formatting.
  virtual void DumpJSON(std::ostream& out,
                        const int         numSpaces = 0) const
  {
    //std::string ws; // whitespace.
    //for ( int k = 0; k < numSpaces; ++k ) ws += " ";

    //char sguid[Standard_GUID_SIZE_ALLOC];
    //this->GetGUID().ToCString(sguid);

    //out << "\n"  << ws.c_str() << "{";
    //out << "\n"  << ws.c_str() << "  \"type\": " << "\"" << this->DynamicType()->Name() << "\"";
    //out << ",\n" << ws.c_str() << "  \"guid\": " << "\"" << sguid                       << "\"";
    //out << ",\n" << ws.c_str() << "  \"name\": " << "\"" << this->GetName()             << "\"";
    ////
    //this->dumpJSON(out, numSpaces + 2);
    ////
    //out << "\n" << ws.c_str() << "}";
  }

  //! Dumps the attribute to the plotter.
  //! \param[in] plotter plotter to dump the attribute.
  /*virtual void DumpGraphically(ActAPI_PlotterEntry plotter) const
  {
    FR_NotUsed(plotter);
  }*/

public:

  //! Hasher for sets.
  struct t_hasher
  {
    static int HashCode(const Handle(FR_FeatureAttr)& attr, const int upper)
    {
        assert(false);
        return 0;
      //return Standard_GUID::HashCode(attr->GetGUID(), upper);
    }

    static bool IsEqual(const Handle(FR_FeatureAttr)& attr, const Handle(FR_FeatureAttr)& other)
    {
      //return Standard_GUID::IsEqual( attr->GetGUID(), other->GetGUID() );
        assert(false);
        return 0;
    }
  };

protected:

  //! Sets back-pointer to AAG.
  //! \param[in] pAAG owner AAG.
  void setAAG(FR_AAG* pAAG)
  {
    m_pAAG = pAAG;
  }

  //! \return back-pointer to the owner AAG.
  FR_AAG* getAAG() const
  {
    return m_pAAG;
  }

protected:

  //! Allows sub-classes to dump additional properties to their JSONs.
  virtual void dumpJSON(std::ostream&, const int) const {}

protected:

  FR_AAG* m_pAAG; //!< Back-pointer to the owner AAG.

};

#endif
