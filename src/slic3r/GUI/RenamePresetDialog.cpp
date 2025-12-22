#include "RenamePresetDialog.hpp"

#include <wx/sizer.h>
#include <wx/stattext.h>

#include <boost/algorithm/string.hpp>

#include "GUI_App.hpp"
#include "Widgets/Label.hpp"
#include "format.hpp"

namespace Slic3r {
namespace GUI {

namespace {
constexpr int kInputWidthDp = 360;
constexpr int kInputHeightDp = 24;
constexpr int kButtonWidthDp = 60;
constexpr int kButtonHeightDp = 24;

wxString type_label(Preset::Type type)
{
    switch (type) {
    case Preset::TYPE_PRINTER:      return _L("printer");
    case Preset::TYPE_FILAMENT:     return _L("material");
    default:                        return _L("preset");
    }
}

std::string trimmed(const std::string &value)
{
    return boost::algorithm::trim_copy(value);
}
} // namespace

RenamePresetDialog::RenamePresetDialog(wxWindow *parent, PresetCollection *collection, const Preset &preset)
    : DPIDialog(parent,
                wxID_ANY,
                _L("Rename preset"),
                wxDefaultPosition,
                wxDefaultSize,
                wxCAPTION | wxCLOSE_BOX),
      m_collection(collection),
      m_type(preset.type),
      m_original_name(preset.name),
      m_new_name(preset.name)
{
    SetBackgroundColour(wxColour(255, 255, 255));

    auto main_sizer = new wxBoxSizer(wxVERTICAL);

    const wxString intro = format_wxstr(_L("Rename %1% preset"), type_label(m_type));
    auto title = new wxStaticText(this, wxID_ANY, intro);
    title->SetFont(Label::Body_14);
    main_sizer->Add(title, 0, wxEXPAND | wxALL, FromDIP(10));

    auto input_sizer = new wxBoxSizer(wxVERTICAL);
    m_input = new TextInput(this,
                            from_u8(m_original_name),
                            wxEmptyString,
                            wxEmptyString,
                            wxDefaultPosition,
                            wxDefaultSize,
                            wxTE_PROCESS_ENTER);
    m_input->SetMinSize(wxSize(FromDIP(kInputWidthDp), FromDIP(kInputHeightDp)));
    m_input->SetMaxSize(wxSize(FromDIP(kInputWidthDp), FromDIP(kInputHeightDp)));
    input_sizer->Add(m_input, 0, wxEXPAND | wxALL, FromDIP(5));
    main_sizer->Add(input_sizer, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP(10));

    m_message = new wxStaticText(this, wxID_ANY, wxEmptyString);
    m_message->SetForegroundColour(wxColour(255, 111, 0));
    main_sizer->Add(m_message, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP(10));

    m_buttons = new DialogButtons(this, {_L("OK"), _L("Cancel")});
    auto ok_button = m_buttons->GetOK();
    ok_button->SetLabel(_L("Rename"));
    ok_button->SetMinSize(wxSize(FromDIP(kButtonWidthDp), FromDIP(kButtonHeightDp)));
    ok_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        if (m_buttons->GetOK()->IsEnabled())
            EndModal(wxID_OK);
    });
    m_buttons->GetCANCEL()->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { EndModal(wxID_CANCEL); });
    main_sizer->Add(m_buttons, 0, wxEXPAND | wxTOP, FromDIP(5));

    SetSizer(main_sizer);
    main_sizer->Fit(this);
    Centre(wxBOTH);

    m_input->GetTextCtrl()->Bind(wxEVT_TEXT, [this](wxCommandEvent &) { update_state(); });
    m_input->GetTextCtrl()->Bind(wxEVT_TEXT_ENTER, [this](wxCommandEvent &) {
        if (m_buttons->GetOK()->IsEnabled())
            EndModal(wxID_OK);
    });

    update_state();
    m_input->GetTextCtrl()->SelectAll();
    m_input->GetTextCtrl()->SetFocus();
    wxGetApp().UpdateDlgDarkUI(this);
}

void RenamePresetDialog::on_dpi_changed(const wxRect &suggested_rect)
{
    Fit();
    if (suggested_rect.IsEmpty())
        CentreOnParent();
}

void RenamePresetDialog::update_state()
{
    wxString error_msg;
    bool valid = true;

    std::string value = into_u8(m_input->GetTextCtrl()->GetValue());
    m_new_name = trimmed(value);

    const std::string &suffix = PresetCollection::get_suffix_modified();
    const char *illegal = "<>[]:/\\|?*\"";

    auto set_error = [&](const wxString &msg) {
        valid = false;
        error_msg = msg;
    };

    if (m_new_name.empty())
        set_error(_L("The name is not allowed to be empty."));
    else if (m_new_name == m_original_name)
        set_error(_L("Enter a different name."));
    else if (m_new_name.front() == ' ')
        set_error(_L("The name is not allowed to start with space character."));
    else if (m_new_name.back() == ' ')
        set_error(_L("The name is not allowed to end with space character."));
    else if (m_new_name.find_first_of(illegal) != std::string::npos)
        set_error(_L("Illegal characters: < > [ ] : / \\ | ? * \""));
    else if (!suffix.empty() && m_new_name.find(suffix) != std::string::npos)
        set_error(format_wxstr(_L("Name is invalid; illegal suffix: %1%"), from_u8(suffix)));
    else if (m_new_name == "Default Setting" || m_new_name == "Default Filament" || m_new_name == "Default Printer")
        set_error(_L("Name is unavailable."));

    if (valid) {
        if (Preset *existing = m_collection ? m_collection->find_preset(m_new_name, false, true) : nullptr) {
            if (existing->name == m_new_name)
                set_error(format_wxstr(_L("Preset \"%1%\" already exists."), from_u8(m_new_name)));
        }
    }

    if (valid && m_collection) {
        if (m_collection->get_preset_name_by_alias(m_new_name) != m_new_name)
            set_error(_L("The name cannot be the same as a preset alias name."));
    }

    m_buttons->GetOK()->Enable(valid);
    m_message->SetLabel(error_msg);
    m_message->Show(!error_msg.IsEmpty());
    Layout();
}

} // namespace GUI
} // namespace Slic3r
