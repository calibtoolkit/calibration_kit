//
// Created by inclo on 2022-03-06.
//

#ifndef CALIBRATION_KIT_TUI_TOOLS_H
#define CALIBRATION_KIT_TUI_TOOLS_H

#include <functional>  // for function
#include <memory>      // for allocator, __shared_ptr_access
#include <string>      // for string, basic_string, operator+, to_string
#include <filesystem>

#include "ftxui/component/captured_mouse.hpp"  // for ftxui
#include "ftxui/component/component.hpp"       // for Menu, Horizontal, Renderer
#include "ftxui/component/component_base.hpp"  // for ComponentBase
#include "ftxui/component/component_options.hpp"  // for MenuOption
#include "ftxui/component/screen_interactive.hpp"  // for Component, ScreenInteractive
#include "ftxui/dom/elements.hpp"  // for text, separator, bold, hcenter, vbox, hbox, gauge, Element, operator|, border

struct BaseTUI
{
    BaseTUI(const std::string& quit_button_name = "Quit")
            :screen{ftxui::ScreenInteractive::TerminalOutput()}
            ,quit{ftxui::Button(quit_button_name, screen.ExitLoopClosure())}
    { }

    ftxui::ScreenInteractive screen;
    ftxui::Component quit;
};

struct ToDoTUI : public BaseTUI
{
    ToDoTUI()
            :BaseTUI("Back")
            ,layout{ftxui::Container::Horizontal({quit})}
    {}

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return ftxui::vbox({ftxui::text("TODO") | ftxui::center, quit->Render()}) | ftxui::border | ftxui::center;
        }));
    }

    ftxui::Component layout;
};

template<typename T>
struct InputHelp;

template<>
struct InputHelp<std::string>
{
    InputHelp()
        : input{ ftxui::Input(&raw_input, &background, &option) }
    { }

    inline operator ftxui::Component()
    { return input; }

    inline auto Render()
    { return ftxui::hbox({ input->Render(), ftxui::text(notice) | ftxui::color(ftxui::Color::Red) }); }

    std::string raw_input;
    std::string background;
    std::string notice;
    ftxui::InputOption option;
    ftxui::Component input;
};

template<typename T>
struct InputHelp : public InputHelp<std::string>
{
    static_assert(std::is_arithmetic_v<T>, "ops");
    static_assert(std::is_same_v<T, std::decay_t<T> >, "ops");
    static_assert(!std::is_same_v<T, bool>, "ops");

    InputHelp()
            : InputHelp<std::string>()
            , value{ }
            , check{ false }
    {
        raw_input = std::to_string(value);
        option.on_change = [this]{ checkInput(); };
    }

    InputHelp(T default_v)
            : InputHelp<std::string>()
            , value{ default_v }
            , check{ true }
    {
        raw_input = std::to_string(value);
        option.on_change = [this]{ checkInput(); };
    }

    explicit operator bool()
    { return check; }

    operator T()
    { return value; }

    //TODO private:
    auto stoAuto(size_t* idx)
    {
        if constexpr(std::is_floating_point_v<T>)
            return std::stod(raw_input, idx);
        else if constexpr(std::is_signed_v<T>)
            return std::stoll(raw_input, idx, 0);
        else
            return std::stoull(raw_input, idx, 0);
    }

    void checkInput()
    {
        if((raw_input.empty() || raw_input.front() > '9' || raw_input.front() < '0') && raw_input.front() != '.')
        {
            if(raw_input.empty())
                notice = "empty!";
            else
                notice = "input error!";
            check = false;
            return;
        }

        size_t count;
        auto temp = stoAuto(&count);
        if(raw_input.size() != count)
        {
            notice = "input error!";
            check = false;
            return;
        }
        value = temp;
        check = true;
        notice.clear();
    }

    T value;
    bool check;
};

struct IOFilesystemBase : private BaseTUI
{
    IOFilesystemBase(std::string& result)
        : BaseTUI("OK")
        , result{ result }
        , last_select{ 0 }
        , file_seletor{ 0 }
        , currten_path{ result }
        , files_menu{ ftxui::Radiobox(&files, &file_seletor, ftxui::RadioboxOption{ "√ ", "  " }) }
    {
        std::string file_name;
        if (std::filesystem::exists(currten_path))
        {
            if (!std::filesystem::is_directory(currten_path))
            {
                file_name = currten_path.filename();
                currten_path = currten_path.parent_path();
            }
        }
        else
            currten_path = std::filesystem::current_path();

        parent_dirs.push_front(currten_path);
        for (auto parent_dir = currten_path.parent_path(); parent_dir.parent_path() != parent_dir; parent_dir = parent_dir.parent_path())
            parent_dirs.push_front(parent_dir);

        parent_buttons = ftxui::Container::Horizontal({ftxui::Button("root", [this]{ currten_path = currten_path.root_path(); }, ftxui::ButtonOption{false})});
        for (auto& dir : parent_dirs)
            parent_buttons->Add(ftxui::Button('/' + dir.filename().string(), [this, dir]{ currten_path = dir; }, ftxui::ButtonOption{false}));

        parent_dirs.push_front(currten_path.root_path());

        findFile();

        if (!file_name.empty())
        {
            auto it = std::find(files.begin(), files.end(), file_name);
            last_select = file_seletor = it - files.begin();
        }

        layout = ftxui::Container::Vertical({parent_buttons, files_menu, quit});
    }

    virtual bool fileFilter(const std::filesystem::directory_entry&)
    { return true; }

    void findFile()
    {
        files.clear();
        for (auto const& dir_entry : std::filesystem::directory_iterator{ currten_path })
        {
            if(!fileFilter(dir_entry))
                continue;
            if(dir_entry.is_directory())
                files.push_back('*' + dir_entry.path().filename().string());
            else
                files.push_back(dir_entry.path().filename());
        }
        std::sort(files.begin(), files.end());
        files.emplace_back("");
        last_select = file_seletor = files.size() - 1;
    }

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            for (size_t i = parent_dirs.size() - 1; currten_path != parent_dirs.back(); --i)
            {
                parent_dirs.pop_back();
                parent_buttons->ChildAt(i)->Detach();
                findFile();
            }

            if(last_select != file_seletor)
            {
                if(files[file_seletor].front() == '*')
                {
                    files[file_seletor].front() = '/';
                    currten_path += files[file_seletor];
                    parent_dirs.push_back(currten_path);
                    auto dir = --parent_dirs.end();
                    parent_buttons->Add(ftxui::Button('/' + parent_dirs.back().filename().string(), [this, dir]{ currten_path = *dir; }, ftxui::ButtonOption{ false }));
                    findFile();
                }
            }

            return ftxui::vbox({
                parent_buttons->Render() | ftxui::border,
                ftxui::separator(),
                files_menu->Render() | ftxui::vscroll_indicator | ftxui::frame | ftxui::size(ftxui::HEIGHT, ftxui::LESS_THAN, 12) | ftxui::border,
                ftxui::separator(),
                quit->Render()}) | ftxui::border | ftxui::center;
        }));
    }


    int last_select;
    int file_seletor;
    std::filesystem::path currten_path;
    std::list<std::filesystem::path> parent_dirs;
    std::vector<std::string> files;
    std::string& result;
    ftxui::Component files_menu;
    ftxui::Component parent_buttons;
    ftxui::Component layout;
};

struct FileSelector : public IOFilesystemBase
{
    FileSelector(std::string& result)
        : IOFilesystemBase(result)
    { }

    ~FileSelector()
    { result = currten_path.string() + '/' + files[file_seletor]; }
};

struct DirSelector : public IOFilesystemBase
{
    DirSelector(std::string& result)
        : IOFilesystemBase(result)
    { }

    virtual bool fileFilter(const std::filesystem::directory_entry& d) override
    { return d.is_directory(); }

    ~DirSelector()
    { result = currten_path.string(); }
};

struct InputFile : public InputHelp<std::string>
{
    InputFile()
        : InputHelp<std::string>()
        , explorer{ ftxui::Button("[explorer]", [this]{ FileSelector{ raw_input }.show(); checkInput(); }, ftxui::ButtonOption{ false }) }
        , check{ false }
    { option.on_change = [this]{ checkInput(); }; }

    explicit operator bool()
    { return check; }

    inline operator ftxui::Component()
    { return ftxui::Container::Horizontal({ input, explorer }); }

    inline auto Render()
    { return ftxui::hbox({ InputHelp<std::string>::Render() | ftxui::flex, explorer->Render() }); }

    void checkInput()
    {
        if (raw_input.empty())
        {
            notice = "Empty";
            check = false;
            return;
        }
        else if(!std::filesystem::exists(raw_input) || std::filesystem::is_directory(raw_input))
        {
            notice = "File does not exist";
            check = false;
            return;
        }
        notice = "";
        check = true;
    }

private:
    ftxui::Component explorer;
    bool check;
};

struct InputDir : public InputHelp<std::string>
{
    InputDir()
        : InputHelp<std::string>()
        , explorer{ ftxui::Button("[explorer]", [this]{ DirSelector{ raw_input }.show(), check = true, notice = ""; }, ftxui::ButtonOption{ false }) }
        , check{ false }
    { option.on_change = [this]{ checkInput(); }; }

    explicit operator bool()
    { return check; }

    inline operator ftxui::Component()
    { return ftxui::Container::Horizontal({ input, explorer }); }

    inline auto Render()
    { return ftxui::hbox({ InputHelp<std::string>::Render() | ftxui::flex, explorer->Render() }); }

    void checkInput()
    {
        if (raw_input.empty())
        {
            notice = "Empty";
            check = false;
            return;
        }
        else if(!std::filesystem::exists(raw_input) || !std::filesystem::is_directory(raw_input))
        {
            notice = "Directory does not exist";
            check = false;
            return;
        }
        notice = "";
        check = true;
    }

private:
    ftxui::Component explorer;
    bool check;
};

#endif //CALIBRATION_KIT_TUI_TOOLS_H
