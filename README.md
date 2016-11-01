#Hostess Robot

Metapackage ROS contenente i pacchetti necessari per il funzionamento del robot hostess. All'interno si trovano:
- il pacchetto [hostess_robot](hostess_robot), creato assieme al metapackage e obbligatorio per il corretto mantenimento del progetto, ma di nessuna utilità pratica;
- un fork del progetto [cob_people_detection](http://wiki.ros.org/cob_people_detection), utilizzato per effettuare il riconoscimento facciale;
- un pacchetto chiamato [hostess_full](hostess_full), che contiene soltanto i file .launch per avviare il robot e i file di visualizzazione mediante rviz;
- un pacchetto chiamato [hostess_skeleton_tracker](hostess_skeleton_tracker), che contiene lo skeleton tracker di OpenNI/NiTE, il controllore del motorino del pan, e il nodo che effettua associazioni tra volti e scheletri e media la veocità del robot in funzione della distanza dell'utente;
- un pacchetto chiamato [hostess_user_registration](hostess_user_registration), che permette di avviare sul robot un piccolo webserver per poter aggiungere e rimuovere destinazioni e utenti, e di poter effettuare il login per poter utilizzare il robot.
