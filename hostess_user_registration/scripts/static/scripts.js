function fixedDigit(number)
{
	var id = "00000000" + number;
	id = id.substring(id.length - 8);
	return id;
}

function startCalibration(button)
{
	disableButton(button);
}

function disableButton(button)
{
	button.disabled = true;
	button.value = 'Calibrazione in corso...';
}

function deleteUsers()
{
	var table = document.getElementById("tabella");
	
	table.deleteRow(2);
	
	//window.confirm('Sei sicuro di voler eliminare gli utenti selezionati?');
}

function selectRow(tr)
{
	if(tr.style.backgroundColor == "red")
	{
		tr.style.backgroundColor = "white";
	}
	else
	{
		tr.style.backgroundColor = "red";
	}
}