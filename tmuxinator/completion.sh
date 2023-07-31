#!/bin/bash

_complete_tmuxinator_ls(){
      # echo $COMP_WORDS

      if [ ${COMP_WORDS[COMP_CWORD-1]} == "tmuxinator" ]
      then 
          COMPREPLY=("start")
      elif [ ${COMP_WORDS[COMP_CWORD-1]} == "start" ]
      then 
          COMPREPLY=("-p")
      elif [ $COMP_CWORD -lt 4 ]
      then
          COMPREPLY=($(ls -1 | grep -- "^${COMP_WORDS[COMP_CWORD]}.*.yml")) #2> /dev/null))
      else
          : 
      fi
      return 0
}

complete -W "tmuxinator" -E 
complete -F _complete_tmuxinator_ls tmuxinator
