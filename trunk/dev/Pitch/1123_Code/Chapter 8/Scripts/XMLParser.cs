using UnityEngine;
using System.Collections;
using System.Xml;

public static class XMLParser {
	private static XmlDocument doc;
	private static XmlNode root;
	
	private static string[] names;
	private static int[] scores;
	private static int userLength;
	
	public static void Parse( string xml) {
		doc = new XmlDocument();
		doc.LoadXml(xml); // Loading from String
		//Using doc.Load("HiScore.xml"); When load from an xml file
		
		//Using Last Child to Skip the <?xml version="1.0" encoding="UTF-8"?>
		//If we load from the xml file we will use the FirstChild instead
		root = doc.LastChild;
		if (root.HasChildNodes) {
			//Getting the Node Length
			userLength = root.ChildNodes.Count;
			names = new string[userLength];
			scores = new int[userLength];
			for (int i = 0; i < userLength; i++) {
				//Getting the user name and scroe XmlAttribute
				XmlAttribute nameAtt = root.ChildNodes[i].Attributes["name"];
				XmlAttribute scoreAtt = root.ChildNodes[i].Attributes["score"];
				//Assigning the user name data to array
				names[i] = (string)nameAtt.Value;
				//Assigning the user score data to array
				scores[i] = ConvertStringtoInt((string)scoreAtt.Value);
			}
		}
	}
	
	//Converting string to int
	private static int ConvertStringtoInt( string s) {
		int j;
		bool result = System.Int32.TryParse(s, out j);
		if (true == result) {
			return j;
		} else {
			Debug.Log("Error...");
		    return 0;
		}
	}
	
	//Getting user name from index
	public static string Name ( int index) {
		return names[index];	
	}
	//Getting user score from index
	public static int Score ( int index) {
		return scores[index];	
	}
	//Getting user length
	public static int UserLength () {
		return userLength;
	}
} 

