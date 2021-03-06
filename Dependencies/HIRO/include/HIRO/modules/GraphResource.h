#pragma once
#include <HIRO/Resource.h>

namespace hiro::modules
{

  class HIRO_API GraphResource : public hiro::Resource
  {
  public:
    GraphResource(const std::string &name);
    void SetData(const std::vector<float> &values_y);
    void SetData(const std::vector<glm::vec2> &values);
    const std::vector<glm::vec2> &GetData() const;
    hiro::PGadget CreateGadget() override;
  private:
    std::vector<glm::vec2> data_;
  };

}
