_ouster_sdk_completer() {
    local -x _DEV_SH_COMPLETE=zsh_complete
    local -x COMP_WORDS="${words[*]}"
    local -x COMP_CWORD=$((CURRENT - 1))

    local -a matches
    local type item help

    while IFS= read -r type && IFS= read -r item && IFS= read -r help; do
        if [[ "$type" == "plain" || "$type" == "completion" ]]; then
            if [[ "$help" == "_" || -z "$help" ]]; then
                matches+=("$item")
            else
                matches+=("${item}:${help}")
            fi
        fi
    done < <("$words[1]" 2>/dev/null)

    if (( ${#matches} > 0 )); then
        _describe 'commands' matches
    else
        _files
    fi
}

compdef _ouster_sdk_completer dev.sh
compdef _ouster_sdk_completer ./dev.sh
compdef _ouster_sdk_completer ./scripts/dev.sh
compdef -P _ouster_sdk_completer '*/dev.sh'
