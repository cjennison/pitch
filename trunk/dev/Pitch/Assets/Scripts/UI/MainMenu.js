

function Start () {

}

 // Tests if the mouse is touching a GUIElement.
    // Add a GUITexture and put the mouse over it and
    // it will print the GUITexture name.
    private var test : GUILayer;
    test = Camera.main.GetComponent(GUILayer);
    function Update() {
        if(test.HitTest(Input.mousePosition) != null) {
           if( Input.GetButtonDown("Fire1") ){
             Debug.Log(test.HitTest(Input.mousePosition).name);
             Application.LoadLevel("Subway");             
           }
        }
    }

function OnMouseDown(){
	Debug.Log(this.name);
}