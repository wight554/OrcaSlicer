#include "RenamedProfilesDialog.hpp"

#include <wx/stattext.h>

#include "GUI_App.hpp"
#include "Widgets/Label.hpp"

namespace Slic3r {
namespace GUI {

namespace {
wxString type_label(Preset::Type type)
{
    switch (type) {
    case Preset::TYPE_PRINTER:  return _L("Printer");
    case Preset::TYPE_FILAMENT: return _L("Material");
    default:                    return _L("Preset");
    }
}
}

RenamedProfilesDialog::RenamedProfilesDialog(wxWindow *parent, const std::vector<RenameUpdateOption> &options)
    : DPIDialog(parent,
                wxID_ANY,
                _L("Update renamed presets"),
                wxDefaultPosition,
                wxDefaultSize,
                wxCAPTION | wxCLOSE_BOX),
      m_options(options)
{
    SetBackgroundColour(wxColour(255, 255, 255));

    auto main_sizer = new wxBoxSizer(wxVERTICAL);

    auto intro = new wxStaticText(this,
                                  wxID_ANY,
                                  _L("The following presets were renamed. Select which ones you would like to update in this project."));
    intro->SetFont(Label::Body_12);
    intro->Wrap(FromDIP(520));
    main_sizer->Add(intro, 0, wxEXPAND | wxALL, FromDIP(10));

    m_list_panel = new wxPanel(this);
    m_list_panel->SetBackgroundColour(wxColour(248, 248, 248));
    auto list_sizer = new wxBoxSizer(wxVERTICAL);
    const int label_width = FromDIP(500);
    for (const auto &option : m_options) {
        auto row = new wxBoxSizer(wxHORIZONTAL);
        auto checkbox = new ::CheckBox(m_list_panel);
        checkbox->SetValue(true);
        row->Add(checkbox, 0, wxALIGN_TOP | wxRIGHT, FromDIP(6));

        auto label = new wxStaticText(m_list_panel, wxID_ANY, describe_option(option), wxDefaultPosition, wxSize(label_width, -1));
        label->Wrap(label_width);
        row->Add(label, 1, wxALIGN_TOP | wxTOP | wxBOTTOM, FromDIP(2));

        list_sizer->Add(row, 0, wxEXPAND | wxALL, FromDIP(6));
        m_checkboxes.push_back(checkbox);
    }
    m_list_panel->SetSizer(list_sizer);
    main_sizer->Add(m_list_panel, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(10));

    m_buttons = new DialogButtons(this, {_L("OK"), _L("Cancel")});
    m_buttons->GetOK()->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { EndModal(wxID_OK); });
    m_buttons->GetCANCEL()->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { EndModal(wxID_CANCEL); });
    main_sizer->Add(m_buttons, 0, wxEXPAND | wxALL, FromDIP(10));

    SetSizer(main_sizer);
    SetMinSize(wxSize(FromDIP(560), FromDIP(240)));
    main_sizer->Fit(this);
    Centre(wxBOTH);

    wxGetApp().UpdateDlgDarkUI(this);
}

std::vector<RenameUpdateOption> RenamedProfilesDialog::selection() const
{
    std::vector<RenameUpdateOption> selected;
    const size_t count = std::min(m_options.size(), m_checkboxes.size());
    for (size_t idx = 0; idx < count; ++idx) {
        if (m_checkboxes[idx]->GetValue())
            selected.push_back(m_options[idx]);
    }
    return selected;
}

void RenamedProfilesDialog::on_dpi_changed(const wxRect &suggested_rect)
{
    if (m_list_panel)
        m_list_panel->Layout();
    Fit();
    if (suggested_rect.IsEmpty())
        CentreOnParent();
}

wxString RenamedProfilesDialog::describe_option(const RenameUpdateOption &option) const
{
    return wxString::Format("%s:\n   %s\n   -> %s",
                            type_label(option.type),
                            from_u8(option.old_name),
                            from_u8(option.new_name));
}

} // namespace GUI
} // namespace Slic3r
