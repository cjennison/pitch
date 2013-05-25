//Public Vars - Displays in the inspector
public var f_speed:float = 5.0; //speed of pitch
public var loopSprites:SpriteManager[]; //controls the update of the sprite animation texture

//Private Vars - Unavailable in the inspector, hidden to other classes
private var in_direction:int;

//Starts as soon as the game begins
public function Start():void {
	in_direction = 1;
	
	//Init the sprite manager
	for(var i:int = 0;i < loopSprites.length;i++){
		//loopSprites[i].init();
	}
	
	//Update Camera to Character Position
	/* Notes
	* transform - refers to the current script's host
	* transform.position - The object that references the objects position(x,y,z);
	
	*/
	Camera.main.transform.position = new Vector3(transform.position.x, transform.position.y, transform.position.z - 20);
}


