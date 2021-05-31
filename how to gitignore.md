# how to use .gitignore

## Examples

'''bash
name.py
*.txt
*.yaml
'''

## Descriptions
untrack files already checked in
'''bash
git rm --cached FILENAME or git rm --cached -r
'''

used to match all files with extension
'''bash
*.extension_name
'''

used to match any file or directory with name
'''bash
**/any_name
'''
 
used to match anything inside directory
'''bash
any_name/**
'''
