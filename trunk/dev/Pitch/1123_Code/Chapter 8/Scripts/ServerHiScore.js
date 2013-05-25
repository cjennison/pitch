//Set the PHP url here
public var PHPUrl : String = "http://www.jatewit.com/Packt/HiScore.php";
//Set the hash key id 
public var hashKey : String = "UNITYGAMEDEVELOPMENTHOTSHOT";

private var obj_WWW : WWWForm;
private var b_loaded : boolean;

public function Start() : void {
	// Empty Check for Inspector values
	if( PHPUrl == "" ) {
		Debug.LogError( "PHP Url cannot be null." );
	}
	if( hashKey == "" ) {
		Debug.LogError( "Hash Key cannot be null." );
	}
}

//Creating the function to send 
public function SendScore( score : int, name : String) : void {
	var w_form : WWWForm = new WWWForm();
	//Telling PHP that the user is submiting the data
	w_form.AddField("action", "PostScore");
	//Sending hash code key to prevent unwanted user
	w_form.AddField("hash", MD5.Md5Sum(name + "-" + score.ToString() + "-" + hashKey)); //Encrypt with MD5
	//Sending the user score
	w_form.AddField("score", score);
	//Sending the user name
	w_form.AddField("name", name);
	//Start waiting for the response back from the server
	StartCoroutine(WaitingForResponse(new WWW(PHPUrl, w_form), null));
}

//Waiting for the response back from the server
public function WaitingForResponse( www : WWW, callback : Function) : IEnumerator {
	yield www;
	
	if (www.error == null) {
		Debug.Log("Successful.");
	} else {
		Debug.Log("Failed.");
	}
	
	if (callback != null) {
		callback(www.text);
		callback = null;
	}
	
	//Clears data
	www.Dispose();
}

//Getting the score from the server
public function GetScores() : void {
	b_loaded = false;
	var w_form : WWWForm = new WWWForm();
	//Telling PHP that the user is loading the data
	w_form.AddField("action", "GetScore");
	//Start waiting for the response back from the server
	StartCoroutine(WaitingForResponse(new WWW(PHPUrl, w_form), LoadXMLData));
}


//Parse the XML data from the server
public function LoadXMLData(string : String) : void {
	XMLParser.Parse(string);
	b_loaded = true;
	Debug.Log(string); 
}

//Getting User length
public function GetUserLength() : int {
	if (XMLParser != null) {
		return XMLParser.UserLength();
	} else {
		return 0;
	}
}
//Getting User Name by index
public function GetNameData(index : int) : String {
	if (XMLParser != null) {
		return XMLParser.Name(index);
	} else {
		return "";
	}
}
//Getting User Score by index
public function GetScoreData(index : int) : int {
	if (XMLParser != null) {
		return XMLParser.Score(index);
	} else {
		return 0;
	}
}
//Loaded XML
public function IsLoaded() : boolean {
	return b_loaded;
} 

