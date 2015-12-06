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
	if(window.confirm('Sei sicuro di voler eliminare gli utenti selezionati?'))
	{
		deleteSelected();
	}
}

function deleteGoals()
{
	if(window.confirm('Sei sicuro di voler eliminare le destinazioni selezionate?'))
	{
		deleteSelected();
	}
}

function deleteSelected()
{
	var checkboxes = document.getElementsByName("checkbox");
	var i;
	
	var table = document.getElementById("table");
	
	for(i = checkboxes.length - 1; i >= 0; i--)
	{
		if(checkboxes[i].checked)
		{
			table.deleteRow(i + 1);
			//Mandare richiesta di eliminazione al database
		}
	}
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