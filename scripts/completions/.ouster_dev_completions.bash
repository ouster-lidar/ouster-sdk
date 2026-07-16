_ouster_sdk_completer() {
    local cur prev words cword
    # Initialize Bash completion variables
    _get_comp_words_by_ref -n : cur prev words cword

    # Click/Python-specific environment variables for Bash
    local -x _DEV_SH_COMPLETE=bash_complete
    local -x COMP_WORDS="${words[*]}"
    local -x COMP_CWORD=$cword

    local matches=()
    local type item help

    # Execute the script and parse the output
    # Bash process substitution syntax is the same as Zsh
    while read -r type && read -r item && read -r help; do
        if [[ "$type" == "plain" || "$type" == "completion" ]]; then
            # Bash doesn't support the 'item:help' description format in the same way
            # as Zsh's _describe. We focus on the completion items.
            matches+=("$item")
        fi
    done < <("${words[0]}" 2>/dev/null)

    # Filter matches based on what the user has already typed (cur)
    COMPREPLY=( $(compgen -W "${matches[*]}" -- "$cur") )

    # If no matches found, default to file completion
    if [[ ${#COMPREPLY[@]} -eq 0 ]]; then
        COMPREPLY=( $(compgen -f -- "$cur") )
    fi
}

# Register the function for dev.sh
complete -F _ouster_sdk_completer dev.sh
complete -F _ouster_sdk_completer ./dev.sh
complete -F _ouster_sdk_completer ./scripts/dev.sh
