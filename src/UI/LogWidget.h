#pragma once
#include <QtGui>
#include <QtWidgets>
#include <fstream>
#include <iostream>


template <class Elem = char, class Tr = std::char_traits<Elem>>
class StandardOutputRedirector : public std::basic_streambuf<Elem, Tr> {
    typedef void (*cb_func_ptr)(const Elem*, std::streamsize count, void* data);

public:
    StandardOutputRedirector(std::ostream& stream,
        cb_func_ptr cb_func,
        void* data)
        : stream_(stream), cb_func_(cb_func), data_(data) {
        buf_ = stream_.rdbuf(this);
    };

    ~StandardOutputRedirector() { stream_.rdbuf(buf_); }

    std::streamsize xsputn(const Elem* ptr, std::streamsize count) {
        cb_func_(ptr, count, data_);
        return count;
    }

    typename Tr::int_type overflow(typename Tr::int_type v) {
        Elem ch = Tr::to_char_type(v);
        cb_func_(&ch, 1, data_);
        return Tr::not_eof(v);
    }

private:
    std::basic_ostream<Elem, Tr>& stream_;
    std::streambuf* buf_;
    cb_func_ptr cb_func_;
    void* data_;
};

class LogWidget : public QWidget {
public:
    explicit LogWidget(QWidget* parent, int max_num_blocks = 100000);
    ~LogWidget();

    void Append(const std::string& text);
    void Flush();
    void Clear();
    void SaveLogToFile(const std::string& filePath);

private:
    static void Update(const char* text,
        std::streamsize count,
        void* text_box_ptr);

    void SaveLog();

    QMutex mutex_;
    std::string text_queue_;
    QPlainTextEdit* text_box_;
    std::ofstream log_file_;
    StandardOutputRedirector<char, std::char_traits<char>>* cout_redirector_;
    StandardOutputRedirector<char, std::char_traits<char>>* cerr_redirector_;
    StandardOutputRedirector<char, std::char_traits<char>>* clog_redirector_;
};