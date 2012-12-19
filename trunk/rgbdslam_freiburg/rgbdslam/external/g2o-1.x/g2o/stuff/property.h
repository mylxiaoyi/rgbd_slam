// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef _G2O_PROPERTY_H_
#define _G2O_PROPERTY_H_

#include <string>
#include <map>
#include <sstream>

#include "string_tools.h"

namespace g2o {

  class BaseProperty {
    public:
      BaseProperty(const std::string name_);
      virtual ~BaseProperty();
      const std::string& name() {return _name;}
      virtual std::string toString() const = 0;
      virtual bool fromString(const std::string& s) = 0;
    protected:
      std::string _name;
  };

  template <typename T>
  class Property: public BaseProperty {
    public:
      typedef T ValueType;
      Property(const std::string& name_): BaseProperty(name_){}
      Property(const std::string& name_, const T& v): BaseProperty(name_), _value(v){}
      void setValue(const T& v) {_value = v; }
      const T& value() const {return _value;}
      virtual std::string toString() const
      {
        std::stringstream sstr;
        sstr << _value;
        return sstr.str();
      }
      virtual bool fromString(const std::string& s)
      {
        bool status = convertString(s, _value);
        return status;
      }
    protected:
      T _value;
  };

  class PropertyMap: public std::map<std::string, BaseProperty*> {
    public:
      typedef std::map<std::string, BaseProperty*>::iterator PropertyMapIterator;

      bool addProperty(BaseProperty* p);

      bool eraseProperty(const std::string& name_);

      template <typename P> 
      P* getProperty(const std::string& name_) {
        PropertyMapIterator it=find(name_);
        if (it==end())
          return 0;
        return dynamic_cast<P*>(it->second);
      }

      template <typename P> 
      P* makeProperty(const std::string& name_, const typename P::ValueType& v) {
        PropertyMapIterator it=find(name_);
        if (it==end()){
          P* p=new P(name_, v);
          addProperty(p);
          return p;
        } else 
          return dynamic_cast<P*>(it->second);
      }

      ~PropertyMap();

  };

  typedef Property<int> IntProperty;
  typedef Property<bool> BoolProperty;
  typedef Property<float> FloatProperty;
  typedef Property<double> DoubleProperty;
  typedef Property<std::string> StringProperty;

} // end namespace
#endif
