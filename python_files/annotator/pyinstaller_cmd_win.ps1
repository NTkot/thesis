#Requires -Version 5.0

# Get the directory of the script
$ScriptDirectory = [System.IO.Path]::GetDirectoryName($MyInvocation.MyCommand.Definition)

# Clean up directories and files
Remove-Item -Path (Join-Path -Path $ScriptDirectory -ChildPath 'pyinstaller_win') -Recurse -Force -ErrorAction SilentlyContinue

# Check for the 'ann_venv' folder
if (-Not (Test-Path -Path (Join-Path -Path $ScriptDirectory -ChildPath 'ann_venv') -PathType Container)) {
    Return "You need to create a Python virtual environment with the necessary dependencies found in requirements.txt in order to run the pyinstaller script."
} else {
    Write-Output "Found 'ann_venv' folder, assuming it is a Python virtual environment folder."
}

.$ScriptDirectory\ann_venv\Scripts\Activate.ps1

.$ScriptDirectory\ann_venv\Scripts\pyinstaller.exe --onefile `
	--add-data $ScriptDirectory\annotator.yaml:. `
	--add-data $ScriptDirectory\ui\annotator_icon.png:.\ui\ `
	--add-data $ScriptDirectory\ui\play.png:.\ui\ `
	--specpath $ScriptDirectory\pyinstaller_win\ `
    --workpath $ScriptDirectory\pyinstaller_win\build `
    --distpath $ScriptDirectory\pyinstaller_win\dist `
	--name annotator `
	--clean $ScriptDirectory\main.py 
	
cd .\pyinstaller_win
.$ScriptDirectory\ann_venv\Scripts\pyinstaller.exe .\annotator.spec
cd ..

deactivate
