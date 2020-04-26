#include<mainwindow.h>

CodeEditor::CodeEditor(QWidget *parent) : QPlainTextEdit (parent)
{
    lineNumberArea = new LineNumberArea(this);

    connect(this, SIGNAL(blockCountChanged(int)), this, SLOT(updateLineNumberAreaWidth(int)));
    connect(this, SIGNAL(updateRequest(QRect, int)), this, SLOT(updateLineNumberArea(QRect, int)));
    connect(this, SIGNAL(cursorPositionChanged()), this, SLOT(highlightCurrentLine()));



    updateLineNumberAreaWidth(0);
    highlightCurrentLine();
}

int CodeEditor::lineNumberAreaWidth()
{
    int digits = 1;
    int max = qMax(1, blockCount());
    while (max >= 10) {
        max /= 10;
        ++digits;
    }

    int space = 3 + fontMetrics().width(QLatin1Char('9')) * digits;

    return space;
}

void CodeEditor::updateLineNumberAreaWidth(int){
    setViewportMargins(lineNumberAreaWidth(), 0, 0, 0);
}

void CodeEditor::updateLineNumberArea(const QRect &rect, int dy)
{
    if(dy)
        lineNumberArea->scroll(0, dy);
    else
        lineNumberArea->update(0, rect.y(), lineNumberArea->width(), rect.height());

    if (rect.contains(viewport()->rect()))
        updateLineNumberAreaWidth(0);
}

void CodeEditor::resizeEvent(QResizeEvent *event)
{
    QPlainTextEdit::resizeEvent(event);

    QRect cr = contentsRect();
    lineNumberArea->setGeometry(QRect(cr.left(), cr.top(), lineNumberAreaWidth(), cr.height()));
}

void CodeEditor::highlightCurrentLine()
{
    QList<QTextEdit::ExtraSelection> extraSelctions;

    if (!isReadOnly()) {
        QTextEdit::ExtraSelection selection;

        QColor lineColor = QColor(Qt::yellow).lighter(160);

        selection.format.setBackground(lineColor);
        selection.format.setProperty(QTextFormat::FullWidthSelection, true);
        selection.cursor = textCursor();
        selection.cursor.clearSelection();
        extraSelctions.append(selection);
    }

    setExtraSelections(extraSelctions);
}

void CodeEditor::lineNumberAreaPaintEvent(QPaintEvent *event)
{
    QPainter painter(lineNumberArea);
    painter.fillRect(event->rect(), Qt::lightGray);

    QTextBlock block = firstVisibleBlock();
    int blockNumber = block.blockNumber();
    int top = static_cast<int>(blockBoundingGeometry(block).translated(contentOffset()).top());
    int bottom = top + static_cast<int>(blockBoundingRect(block).height());

    while (block.isValid() && top <= event->rect().bottom()) {
        if (block.isVisible() && bottom >= event->rect().top()) {
            QString number = QString::number(blockNumber + 1);
            painter.setPen(Qt::black);
            painter.drawText(0, top, lineNumberArea->width(), fontMetrics().height(), Qt::AlignRight, number);
        }

        block = block.next();
        top = bottom;
        bottom = top + static_cast<int>(blockBoundingRect(block).height());
        ++blockNumber;
    }
}


Highlighter::Highlighter(QTextDocument *parent) : QSyntaxHighlighter (parent)
{
    HighlightingRule rule;

    keywordFormat.setForeground(Qt::darkBlue);
    keywordFormat.setFontWeight(QFont::Bold);
    QStringList keywordPatterns;

    keywordPatterns << "\\basm\\b" << "\\belse\\b" << "\\bnew\\b" << "\\bthis\\b" <<  "\\bauto\\b" << "\\benum\\b"
                    << "\\boperator\\b" << "\\bthrow bool\\b" << "\\bexplicit\\b" << "\\bprivate\\b" << "\\btrue\\b"
                    << "\\bbreak\\b" << "\\bexport\\b" << "\\bprotected\\b" << "\\btry\\b" << "\\bcase\\b" << "\\bextern\\b"
                    << "\\bpublic\\b" << "\\btypedef\\b" << "\\bcatch\\b" << "\\bfalse\\b" << "\\bregister\\b" << "\\btypeid\\b"
                    << "\\bchar\\b" << "\\bfloat\\b" << "\\breinterpret_cast\\b" << "\\btypename\\b" << "\\bclass\\b"
                    << "\\bfor\\b" << "\\breturn\\b" << "\\bunion\\b" << "\\bconst\\b" << "\\bfriend\\b" << "\\bshort\\b"
                    << "\\bunsigned\\b" << "\\bconst_cast\\b" << "\\bgoto\\b" << "\\bsigned\\b" << "\\busing\\b"
                    << "\\bcontinue\\b" << "\\bif\\b" << "\\bsizeof\\b" << "\\bvirtual\\b" << "\\bdefault\\b"
                    << "\\binline\\b" << "\\bstatic\\b" << "\\bvoid\\b" << "\\bdelete\\b" << "\\bint\\b"
                    << "\\bstatic_cast\\b" << "\\bvolatile\\b" << "\\bdo\\b" << "\\blong\\b" << "\\bstruct\\b"
                    << "\\bwchar_t\\b" << "\\bdouble\\b" << "\\bmutable\\b" << "\\bswitch\\b" << "\\bwhile\\b"
                    << "\\bdynamic_cast\\b" << "\\bnamespace\\b" << "\\btemplate\\b" << "\\band\\b" << "\\bbitor\\b"
                    << "\\bnot_eq\\b" << "\\bxor\\b" << "\\band_eq\\b" << "\\bcompl\\b" << "\\bor\\b" << "\\bxor_eq\\b"
                    << "\\bbitand\\b" << "\\bnot\\b" << "\\bor_eq\\b" << "\\bbool\\b";
    foreach (const QString &pattern, keywordPatterns) {
        rule.pattern = QRegularExpression(pattern);
        rule.format = keywordFormat;
        highlightingRules.append(rule);
    }

    classFormat.setFontWeight(QFont::Bold);
    classFormat.setForeground(Qt::darkMagenta);
    rule.pattern = QRegularExpression("\\bQ[A-Za-z]+\\b");
    rule.format = classFormat;
    highlightingRules.append(rule);

    quotationFormat.setForeground(Qt::darkGreen);
    rule.pattern = QRegularExpression("\".*\"");
    rule.format = quotationFormat;
    highlightingRules.append(rule);

    functionFormat.setFontItalic(true);
    functionFormat.setForeground(Qt::blue);
    rule.pattern = QRegularExpression("\\b[A-Za-z0-9_]+(?=\\()");
    rule.format = functionFormat;
    highlightingRules.append(rule);

    singleLineCommentFormat.setForeground(Qt::gray);
    singleLineCommentFormat.setFontItalic(true);
    rule.pattern = QRegularExpression("//[^\n]*");
    rule.format = singleLineCommentFormat;
    highlightingRules.append(rule);

    singleLineCommentFormat.setForeground(Qt::green);
    singleLineCommentFormat.setFontItalic(false);
    rule.pattern = QRegularExpression("#include[^\n]*");
    rule.format = singleLineCommentFormat;
    highlightingRules.append(rule);

    multiLineCommentFormat.setForeground(Qt::gray);
    multiLineCommentFormat.setFontItalic(true);
    commentStartExpression = QRegularExpression("/\\*");
    commentEndExpression = QRegularExpression("\\*/");
}

void Highlighter::highlightBlock(const QString &text)
{
    foreach (const HighlightingRule &rule, highlightingRules) {
        QRegularExpressionMatchIterator matchIterator = rule.pattern.globalMatch(text);
        while (matchIterator.hasNext()) {
            QRegularExpressionMatch match = matchIterator.next();
            setFormat(match.capturedStart(), match.capturedLength(), rule.format);
        }
    }
    setCurrentBlockState(0);

    int startIndex = 0;
    if (previousBlockState() != 1)
        startIndex = text.indexOf(commentStartExpression);

    while (startIndex >= 0) {
        QRegularExpressionMatch match = commentEndExpression.match(text, startIndex);
        int endIndex = match.capturedStart();
        int commentLength = 0;
        if (endIndex == -1) {
            setCurrentBlockState(1);
            commentLength = text.length() - startIndex;
        } else {
            commentLength = endIndex - startIndex + match.capturedLength();
        }
        setFormat(startIndex, commentLength, multiLineCommentFormat);
        startIndex = text.indexOf(commentStartExpression, startIndex + commentLength);
    }
}

notePad::notePad(QWidget *parent) : QWidget(parent)
{
    editor = new CodeEditor();

    QFont font;
    font.setFamily("Courier");
    font.setStyleHint(QFont::Monospace);
    font.setFixedPitch(true);
    font.setPointSize(12);

    editor->setFont(font);

    const int tabStop = 4;
    QFontMetrics metrics(font);
    editor->setTabStopWidth(tabStop * metrics.width(' '));
    highlighter = new Highlighter(editor->document());
	/*usleep(500000);
	filename = "/home/justin/programs/cpp/GUI/cache/current/camera.cpp";
	QFile file(filename);
    if(!file.open(QIODevice::ReadOnly | QFile::Text)){
        QMessageBox::warning(this, "Warning", "Cannot open file : " + file.errorString());
        return;
    }
    QTextStream in(&file);
    QString text = in.readAll();
    editor->setPlainText(text);
    file.close();*/
}	

void notePad::main_c()
{   
    int input;
    /*QFile file(filename);
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        QMessageBox::warning(this, "Warning", "Cannot save file : " + file.errorString());
        return;
    }
    QTextStream out(&file);
    QString text = editor->toPlainText();
    out << text;
    file.close();*/
    input = system("gnome-terminal -e 'scp /home/justin/programs/cpp/GUI/cache/current/main.cpp alarm@192.168.7.18:/home/alarm/project/cpp/camera4/'");
}

void notePad::camera_c()
{
    int input;
    input = system("gnome-terminal -e 'scp /home/justin/programs/cpp/GUI/cache/current/camera.cpp alarm@192.168.7.18:/home/alarm/project/cpp/camera4/'");
}

void notePad::camera_h()
{
    int input;
    input = system("gnome-terminal -e 'scp /home/justin/programs/cpp/GUI/cache/current/camera.h alarm@192.168.7.18:/home/alarm/project/cpp/camera4/'");
}

void notePad::config()
{
    int input;
    input = system("gnome-terminal -e 'scp /home/justin/programs/cpp/GUI/cache/current/config.txt alarm@192.168.7.18:/home/alarm/project/cpp/camera4/'");
}

void notePad::debug_c()
{
    int input;
    input = system("gnome-terminal -e 'scp /home/justin/programs/cpp/GUI/cache/current/debug.cpp alarm@192.168.7.18:/home/alarm/project/cpp/camera4/'");
}
