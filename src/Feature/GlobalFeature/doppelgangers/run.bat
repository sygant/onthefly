@echo off

set PYTHONEXE=D:\GWT\On_the_fly_SfME\thirdparty\Python\python.exe
set IMAGEDIR=D:\Dataset\AmbiguousTextures\radcliffe_camera\amerge\images
set OUTPUTDIR=D:\ECCV\SP_LG_Similar\camera

setlocal enabledelayedexpansion

set CONFIG="configs\test_configs\disambiguation_example.yaml"

REM 创建配置文件
(
    echo data:
    echo   output_path: %OUTPUTDIR%
    echo   type: doppelgangers.datasets.sfm_disambiguation_dataset
    echo   num_workers: 4
    echo   image_dir: %IMAGEDIR%
    echo   loftr_match_dir: %OUTPUTDIR%\loftr_match
    echo   test:
    echo     batch_size: 1
    echo     img_size: 1024
    echo     pair_path: %OUTPUTDIR%\pairs_list.npy
    echo.
    echo models:
    echo   decoder:
    echo     type: doppelgangers.models.cnn_classifier
    echo     input_dim: 10
) > "%CONFIG%"


 %PYTHONEXE% ../script_sfm_disambiguation.py ^
    %CONFIG% ^
    --colmap_exe_command ^
    colmap ^
    --skip_feature_matching ^
    --matching_type ^
    exhaustive_matcher ^
    --input_image_path ^
    %IMAGEDIR% ^
    --output_path ^
    %OUTPUTDIR% ^
    --database_path ^
    %OUTPUTDIR%\1.db ^
    --skip_reconstruction ^
    --threshold ^
    0.99

pause