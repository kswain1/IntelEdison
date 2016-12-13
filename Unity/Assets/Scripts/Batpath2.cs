using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.UI;
using System.IO;
using System;

public class Batpath2 : MonoBehaviour 
{
	public TextAsset csvFile; // Reference of CSV file
	public InputField rollNoInputField;// Reference of rollno input field
	public InputField nameInputField; // Reference of name input filed
	public Text contentArea; // Reference of contentArea where records are displayed
	private char lineSeperater = '\n'; // It defines line seperate character
	private char fieldSeperator = ','; // It defines field seperate chracter
	//public string pathName = "/Users/isisashford/Downloads/IntelEdison-master\ 3/euler_parametrization_animation/accel_roll_pitch_demo.csv";
	//public FileStream fileStream = new FileStream(@"\\Users\\isisashford\\Downloads\\IntelEdison-master\\ 3/\\euler_parametrization_animation\\accel_roll_pitch_demo.csv",FileMode.Open, FileAccess.Read);

//	public string currentDirectory = Directory.GetCurrentDirectory();
	void Start ()
	{
		readData ();
	}

	// Read data from CSV file
	private void readData()
	{

			
		string[] records = csvFile.text.Split (lineSeperater);
		foreach (string record in records)
		{
			string[] fields = record.Split(fieldSeperator);
			foreach(string field in fields)
			{
				contentArea.text += field + "\t";
			}
			contentArea.text += '\n';
		}
	}



	
	// Update is called once per frame
	void Update () {
		
	}
}
