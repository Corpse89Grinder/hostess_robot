#!/bin/bash

if [[ ! -e $HOME/.ros/hostess_user_registration/database.sqlite ]]; then
    mkdir $HOME/.ros/hostess_user_registration
else
    echo "Database già presente nella directory $HOME/.ros/hostess_user_registration/, se si sta resettando l'applicazione si prega di rimuovere il database prima di procedere nuovamente."
    exit 1
fi

cd scripts

cat << EOF | python -
from app import db
db.create_all()
EOF

cd ..

echo "Database creato correttamente."

exit 0
