<?PHP

// Connect to Database
$link = mysql_connect( "your host", "your username", "your password" ) or die( mysql_error() );
mysql_select_db( "your database name" ) or die( mysql_error() );

// Get Data
$name = $_POST['name']; //Get name from Unity
$score = $_POST['score']; //Get score from Unity
$action = $_POST[ 'action' ]; //Get request action from Unity
$unityHash = $_POST[ 'hash' ]; //Get hash key from Unity

//Change this value to match the value stored in the client javascript below
$secretKey="UNITYGAMEDEVELOPMENTHOTSHOT"; 

// Set locally, used to match with hash from Unity
$phpHash = md5($name."-".$score."-".$secretKey);

switch ( $action )
{
	case "GetScore":
		GetScores();
		break;
	case "PostScore":
		if( $phpHash == $unityHash ) {
			PostScore();
		}
		break;
	default:
		GetScores();
		break;
}

////////////////////////////////////////////////////////////////////////////////////////
function PostScore()
{		 
	$score = $_POST[ 'score' ]; //score pass from Unity
	$name  = $_POST[ 'name' ]; //name pass from Unity
	
	//scores = the name of your table in the database (can be anything)
	//name = the first database field (can be anything)
	//score = the second database field (can be anything)
	$query = "INSERT INTO scores ( name, score ) VALUES ( '" . $name . "', '" . $score . "')";
	
	mysql_query( $query ) or die( mysql_error() );
}
////////////////////////////////////////////////////////////////////////////////////////
function GetScores()
{	
	if( $_POST[ 'size' ] != "" ) {
		$size = $_POST[ 'size' ];
	} else {
		$size = 10;
	}
	//scores = the name of your table in the database (can be anything)
	//name = the first database field (can be anything)
	//score = the second database field (can be anything)
	
	$query = "SELECT * FROM scores ORDER BY score DESC LIMIT " . $size;
	$results = mysql_query( $query ) or die( mysql_error() );
	
	//Return XML String to Unity
	echo "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
	echo "<scores>\n";
	if( mysql_num_rows( $results ) > 0 )
	{	
		while( $line = mysql_fetch_array( $results ) ) {
			echo "	<user name=\"" . $line["name"] . "\" " . "score=\"" . $line["score"] . "\" />\n";
		}
	}
	else {
		echo "No entries yet.";
	}
	echo "</scores>\n";
}
////////////////////////////////////////////////////////////////////////////////////////
// Close mySQL Connection
mysql_close($link);
?>