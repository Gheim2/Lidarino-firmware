#!/usr/bin/env bash
# Sincronizza il firmware (PC Windows) -> Banana Pi ~/Lidarino-firmware con rsync,
# per compilare e caricare il firmware direttamente dal Pi via SSH.
#
# Uso (da Git Bash):
#   ./sync_to_pi.sh             copia/aggiorna i file, non cancella nulla sul Pi
#   ./sync_to_pi.sh --dry-run   mostra cosa verrebbe trasferito, senza toccare il Pi
#   ./sync_to_pi.sh --delete    cancella anche sul Pi i file rimossi in locale
#                               (mostra prima l'anteprima e chiede conferma)
#
# Mai toccati sul Pi (né copiati né cancellati):
#   .pio/                  (build e librerie di PlatformIO, il Pi ha le sue)
#   .git/ .vscode/
#   CLAUDE.md .claude/
#   tutto ciò che è in .gitignore
#
# Variabili opzionali: LIDARINO_HOST (default: bananapi, da ~/.ssh/config)
#                      LIDARINO_FW_REMOTE_DIR (default: Lidarino-firmware, relativo alla home)
set -euo pipefail

REMOTE_HOST="${LIDARINO_HOST:-bananapi}"
REMOTE_DIR="${LIDARINO_FW_REMOTE_DIR:-Lidarino-firmware}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

dry_run=false
delete=false
for arg in "$@"; do
    case "$arg" in
        -n|--dry-run) dry_run=true ;;
        --delete)     delete=true ;;
        -h|--help)    sed -n '2,19p' "$0"; exit 0 ;;
        *) echo "Argomento sconosciuto: $arg (usa --help)" >&2; exit 2 ;;
    esac
done

if ! command -v rsync >/dev/null; then
    echo "rsync non trovato nel PATH (atteso in ~/bin)." >&2
    exit 1
fi

cd "$REPO_DIR"

# Il repo è LF (.gitattributes): un file salvato in CRLF da un editor Windows
# verrebbe copiato così com'è, quindi meglio fermarsi e segnalarlo.
crlf_files="$(git ls-files --eol -co --exclude-standard | awk -F'\t' '$1 ~ /w\/crlf/ { print $2 }')"
if [ -n "$crlf_files" ]; then
    echo "ERRORE: questi file hanno fine riga CRLF, convertili in LF prima di sincronizzare:" >&2
    echo "$crlf_files" | sed 's/^/  /' >&2
    echo "Per convertirli: sed -i 's/\r\$//' <file>   (oppure in VS Code: 'CRLF' -> 'LF' in basso a destra)" >&2
    exit 1
fi

rsync_opts=(
    -rlptz
    --prune-empty-dirs
    --itemize-changes
    --human-readable
    --exclude=/.pio/
    --exclude=.git/
    --exclude=.vscode/
    --exclude=CLAUDE.md
    --exclude=.claude/
    --filter=':- .gitignore'
)
# I file esclusi non vengono mai cancellati sul Pi (non usiamo --delete-excluded)
$delete && rsync_opts+=(--delete)

target="$REMOTE_HOST:$REMOTE_DIR/"

if $dry_run; then
    echo ">> Anteprima (nessuna modifica): $REPO_DIR/ -> $target"
    rsync "${rsync_opts[@]}" --dry-run ./ "$target"
    exit 0
fi

if $delete; then
    echo ">> Anteprima dei file che verrebbero CANCELLATI su $target:"
    deletions="$(rsync "${rsync_opts[@]}" --dry-run ./ "$target" | grep '^\*deleting' || true)"
    if [ -z "$deletions" ]; then
        echo "   nessuno"
    else
        echo "$deletions"
        read -r -p "Confermi la cancellazione? [s/N] " answer
        [[ "$answer" =~ ^[sSyY]$ ]] || { echo "Annullato."; exit 1; }
    fi
fi

echo ">> Sincronizzazione: $REPO_DIR/ -> $target"
rsync "${rsync_opts[@]}" ./ "$target"
echo ">> Fatto. Sul Pi: cd ~/$REMOTE_DIR && pio run   (upload solo dopo conferma: pio run -t upload)"
