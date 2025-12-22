#ifndef slic3r_RenamePresetDialog_hpp_
#define slic3r_RenamePresetDialog_hpp_

#include <wx/dialog.h>

#include "wxExtensions.hpp"
#include "libslic3r/Preset.hpp"
#include "Widgets/DialogButtons.hpp"
#include "Widgets/TextInput.hpp"

class wxStaticText;

namespace Slic3r {
namespace GUI {

class RenamePresetDialog : public DPIDialog
{
public:
    RenamePresetDialog(wxWindow *parent, PresetCollection *collection, const Preset &preset);

    const std::string& new_name() const { return m_new_name; }

protected:
    void on_dpi_changed(const wxRect &suggested_rect) override;
    void on_sys_color_changed() override {}

private:
    void update_state();

    PresetCollection *m_collection { nullptr };
    Preset::Type      m_type       { Preset::TYPE_INVALID };
    std::string       m_original_name;
    std::string       m_new_name;

    TextInput    *m_input   { nullptr };
    wxStaticText *m_message { nullptr };
    DialogButtons *m_buttons { nullptr };
};

} // namespace GUI
} // namespace Slic3r

#endif /* slic3r_RenamePresetDialog_hpp_ */
