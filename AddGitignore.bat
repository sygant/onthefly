@echo off
setlocal enabledelayedexpansion

:: 检查是否存在 .gitignore 文件，如果不存在则创建
if not exist .gitignore (
    echo. > .gitignore
)

:: 查找大于 10MB 的文件
for /r %%F in (*) do (
    set "size=0"
    set /a "size = %%~zF / 1024 / 1024"

    if !size! gtr 10 (
        :: 获取相对路径
        set "filePath=%%~dpnF"
        set "relativePath=!filePath:%cd%\=!"
        
        :: 追加到 .gitignore 文件
        echo !relativePath!%%~xF >> .gitignore
    )
)

endlocal