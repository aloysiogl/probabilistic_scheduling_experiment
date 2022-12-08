# Read org-mode file and export to pdf
# Usage: ./compile.sh foo.org

org_file=$1

emacs \
    -u "$(id -un)" \
    --batch \
    --eval '(load user-init-file)' \
    $org_file \
    -f org-latex-export-to-pdf
