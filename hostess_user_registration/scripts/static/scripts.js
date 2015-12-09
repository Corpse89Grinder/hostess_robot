function fixedDigit(number)
{
	var id = "00000000" + number;
	id = id.substring(id.length - 8);
	return id;
}

function toggle(source)
{
	var checkboxes = document.getElementsByName("checkbox");

	var i;
	
	for(i = 0; i < checkboxes.length; i++)
	{
		checkboxes[i].checked = source.checked;
	}
}

function onToggle()
{
	var checkbox = document.getElementById("selectAll");
	
	var checkboxes = document.getElementsByName("checkbox");
	
	var checked = 0;
	var unchecked = 0;
	
	for(var i = 0; i < checkboxes.length; i++)
	{
		if(checkboxes[i].checked)
		{
			checked++;
		}
		else
		{
			unchecked++;
		}
	}
	
	if(unchecked == 0)
	{
		checkbox.indeterminate = false;
		checkbox.checked = true;
	}
	else if(checked == 0)
	{
		checkbox.indeterminate = false;
		checkbox.checked = false;
	}
	else
	{
		checkbox.indeterminate = true;
	}
}

function setImageSource()
{
	image = document.getElementById("stream");
	
	image.src = 'http://' + location.hostname + ':8080/stream?topic=/robot/rgb/image_raw';
}

function startCalibration(button, name, surname, goal, mail)
{
	disableButton(button);
	button.value = 'In attesa del server...';
	
	var namespace = '/new_user/calibration';
	
	var socket = io.connect('http://' + document.domain + ':' + location.port + namespace);
	
	var timeout;
	
	socket.on('started', function()
	{
		clearTimeout(timeout);
		button.value = 'Calibrazione in corso...';
	});
	
	socket.on('progress', function(msg)
	{
		$('#progress').val(msg.images_captured);
	});
	
	socket.on('calibrated', function()
	{
		button.value = 'Aggiornamento robot in corso...';
		socket.emit('credentials', {nome: name, cognome: surname, destinazione: goal, email: mail});
	});
	
	socket.on('saved', function(msg)
	{
		button.value = 'Calibrazione completata';
		
		if(msg.status == 0)
		{
			window.alert('Calibrazione completata correttamente.\nUtente aggiunto al database.');
		}
		else if(msg.status == 1 || msg.status == 2)
		{
			window.alert('L\'utente è stato calibrato e aggiunto correttamente\nma non è stato possibile aggiornare il robot.\nRiavviare il robot per caricare il modello aggiornato.');
		}
		
		window.location='/users';
	});
	
	socket.on('failed', failedUserAdd);
	
	socket.emit('start');
	
	timeout = window.setTimeout(failedUserAdd, 10000);
}

function failedUserAdd()
{
	window.alert('Impossibile aggiungere l\'utente al robot.\nControllare che il servizio sia attivo e riprovare.');
	window.location='/users';
}

function disableButton(button)
{
	button.disabled = true;
}

function enableButton(button)
{
	button.disabled = false;
}

function checkEnablingAll(lenght, size)
{
	checkEnabling(lenght);
	
	if(size == 0)
	{
		var button = document.getElementById("add-user-button");
		
		button.onclick = showAlert;
	}
}

function showAlert()
{
	window.alert('Nessuna destinazione in memoria.\nAggiungi qualche destinazione prima di\npoter inserire degli utenti!');
}

function checkEnabling(length)
{
	if(length == 0)
	{
		var table = document.getElementById("table");
		
		table.hidden = true;
		
		var button = document.getElementById("remove-button");
		
		disableButton(button);
	}
}

function deleteUsers()
{
	deleteSelected('Users');
}

function deleteGoals()
{
	deleteSelected('Goals');
}

function deleteSelected(type)
{
	var message;
	var text;
	var xhttp = new XMLHttpRequest();
	
	if(type == 'Users')
	{
		message = 'Sei sicuro di voler eliminare gli utenti selezionati?';
		text = '{ "utenti" : [';
		xhttp.open("POST", "/delete_user_entries", true);
	}
	else if(type == 'Goals')
	{
		message = 'Sei sicuro di voler eliminare le destinazioni selezionate?';
		text = '{ "destinazioni" : [';
		xhttp.open("POST", "/delete_goal_entries", true);
	}
	
	var checkboxes = document.getElementsByName("checkbox");
	
	var i;
	
	var checked = 0;
	
	for(i = 0; i < checkboxes.length; i++)
	{
		if(checkboxes[i].checked)
		{
			checked++;
			text += checkboxes[i].id.slice(checkboxes[i].id.lastIndexOf('-') + 1) + ',';
		}
	}
	
	text = text.slice(0, -1);
	text += '] }';
	
	xhttp.onreadystatechange = function()
	{
		if(xhttp.readyState == 4 && xhttp.status == 200)
		{
			if(xhttp.responseText == 'ok')
			{
				var table = document.getElementById("table");
			
				for(i = checkboxes.length - 1; i >= 0; i--)
				{
					if(checkboxes[i].checked)
					{
						table.deleteRow(i + 1);
					}
				}
				
				var checkbox = document.getElementById("selectAll");
				
				checkbox.indeterminate = false;
				checkbox.checked = false;
				
				if(table.rows.length == 1 && type == 'Users')
				{
					window.location = '/users';
				}
				else if(table.rows.length == 1 && type == 'Goals')
				{
					window.location = '/goals';
				}
			}
			else if(xhttp.responseText == 'failed')
			{
				window.alert('Impossibile eliminare gli utenti dal robot.\nControllare che il servizio sia attivo e riprovare.');
				window.location='/users';
			}
		}
	};
	
	if(checked != 0 && window.confirm(message))
	{
		xhttp.setRequestHeader('Content-Type', 'application/json; charset=UTF-8');
		xhttp.send(text);
	}
}
