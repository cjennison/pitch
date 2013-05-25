//For toggle the open and close our menu window
//We made it static because we can access this variable from everywhere.
public static var b_openMenu : boolean;

public var customSkin : GUISkin; //We assign our MenuSkin here
public var t_hero : Texture;	//Character background texture
public var t_statusBox1 : Texture; //First Info box background texture
public var t_statusBox2 : Texture; //Second Info box background texture
public var t_skillBox : Texture; //Skill box background texture

public var fullHP : int = 9999; //The current full HP
public var fullMP : int = 999; //The current full MP
public var currentHP : int = 9999; //The current HP
public var currentMP : int = 999; //The current MP
public var currentLV : int = 99; //The current LV
public var currentEXP : int = 9999999; //The current EXP
public var currentNEXT : int = 99999; //The current NEXT
public var currentATK : int = 999; //The current ATK
public var currentDEF : int = 999; //The current DEF
public var currentAGI : int = 999; //The current AGI
public var currentINT : int = 999; //The current INT
public var currentLUC : int = 999; //The current LUC

public var a_weapons : Items[]; //weapons array that the character currently has
public var a_armors : Items[]; //armors array that the character currently has
public var a_accessories : Items[]; //accessories array that the character currently has
public var a_items : Items[]; //items array that the character currently has
public var a_skills : Texture[]; //skills array that the character currently has

private var currentWeapon : Items; //current weapon that character uses
private var currentArmor : Items; //current armor that character uses
private var currentAccessory : Items; //current accessory that character uses
private var currentItem : Items; //current item that character uses
private var currentSkill : Texture; //current skill that character uses

private var in_toolbar : int = 0;
private var s_toolbars : String[] = ["STATUS", "INVENTORY", "EQUIPMENT"];
private var r_hero : Rect = new Rect (19, 35, 225, 441);
private var r_window : Rect = new Rect (10, 10, 640, 480);
private var r_closeBtn : Rect = new Rect (598, 8, 26, 22);
private var r_tabButton : Rect = new Rect (35, 15, 480, 40);

private var s_unequip : String = "UNEQUIP";

//Status Tab
private var maxHP : int = 9999; //Maximum limit of HP
private var maxMP : int = 999; //Maximum limit of MP
private var maxLV : int = 99; //Maximum limit of LV
private var maxEXP : int = 9999999; //Maximum limit of EXP
private var maxNEXT : int = 99999; //Maximum limit of NEXT
private var maxATK : int = 999; //Maximum limit of ATK
private var maxDEF : int = 999; //Maximum limit of DEF
private var maxAGI : int = 999; //Maximum limit of AGI
private var maxINT : int = 999; //Maximum limit of INT
private var maxLUC : int = 999; //Maximum limit of LUC

//Rect position for the GUI
private var r_statTexture1 : Rect = new Rect (252, 77, 331, 125);
private var r_statTexture2 : Rect = new Rect (252, 244, 331, 142);
private var r_hpLabel : Rect = new Rect (313, 75, 120, 25);
private var r_mpLabel : Rect = new Rect (313, 100, 120, 25);
private var r_lvLabel : Rect = new Rect (313, 124, 120, 25);
private var r_expLabel : Rect = new Rect (313, 150, 120, 25);
private var r_nextLabel : Rect = new Rect (313, 177, 120, 25);
private var r_atkLabel : Rect = new Rect (529, 75, 50, 25);
private var r_defLabel : Rect = new Rect (529, 100, 50, 25);
private var r_agiLabel : Rect = new Rect (529, 124, 50, 25);
private var r_intLabel : Rect = new Rect (529, 150, 50, 25);
private var r_lucLabel : Rect = new Rect (529, 177, 50, 25);
private var r_statBox : Rect = new Rect (237, 67, 360, 147);
private var r_weaponBox : Rect = new Rect (237, 230, 360, 207);
private var r_weaponLabel : Rect = new Rect (252, 264, 180, 40);
private var r_armorLabel : Rect = new Rect (252, 324, 180, 40);
private var r_accessLabel : Rect = new Rect (252, 386, 180, 40);
private var r_skillTexture : Rect = new Rect (464, 288, 119, 117);
private var r_skillBox : Rect = new Rect (460, 284, 127, 125);
//GUIContent
private var gui_weaponCon : GUIContent;
private var gui_armorCon : GUIContent;
private var gui_accessCon : GUIContent;
private var gui_skillCon : GUIContent;

//Item Tab
private var r_itemsBox : Rect = new Rect (237, 67, 360, 247);
private var r_tipBox : Rect = new Rect (237, 330, 360, 107);
private var r_itemsButton : Rect = new Rect (257, 87, 340, 227);
private var r_tipButton : Rect = new Rect (257, 350, 340, 87);
private var r_verScroll : Rect = new Rect (600, 87, 20, 227);
private var f_scrollPos : float = 1.0;
private var scrollPosition : Vector2 = Vector2.zero;
private var scrollPosition2 : Vector2 = Vector2.zero;
private var in_toolItems : int = 0;

//Equip tab
private var r_equipBox : Rect = new Rect (237, 67, 360, 207);
private var r_equipWeaponBox : Rect = new Rect (237, 280, 360, 157);
private var r_statTextureEquip : Rect = new Rect (252, 81, 331, 142);
private var r_skillBoxEquip : Rect = new Rect (460, 121, 127, 125);

//The position of each equip button from 0 - waepon, 1 - armor, 2 - accessory, 3 - skill
private var r_equipRect : Rect[] = [new Rect (252, 101, 180, 40), new Rect (252, 161, 180, 40), 
									new Rect (252, 221, 180, 40), new Rect (464, 125, 119, 117)];
private var r_equipWindow : Rect = new Rect (500, 0, 70, 100);
private var scrollPosition3 : Vector2 = Vector2.zero;
private var scrollPosition4 : Vector2 = Vector2.zero;
private var scrollPosition5 : Vector2 = Vector2.zero;
private var scrollPosition6 : Vector2 = Vector2.zero;
private var a_equipBoolean : boolean[] = new boolean[4];
private var in_toolWeapons : int = 0;
private var in_toolArmors : int = 0;
private var in_toolAccess : int = 0;
private var in_toolskill : int = 0;

//Items class to contain our information
class Items {
	public var icon : Texture;
	public var name : String;
	public var amount : int;
	
	private var itemName : String;
	
	//This function is just to put the space between name of the item and amount of the item
	public function setUpItemName () : void {
		var in_length : int = (this.name.Length + this.amount.ToString().Length);
		if (in_length < 25) {
			while (this.name.Length < 17 ) {
				this.name += " ";
			}
		}
		if(this.amount < 10) {
			itemName = (this.name + " " + this.amount.ToString());
		} else {
			itemName = (this.name + this.amount.ToString());
		}	
	}
	
	public function get itemNA () : String {
		return itemName;
	}
}

public function Start () : void {
	b_openMenu = false; //Set our menu disabled at the first run
	
	gui_weaponCon = GUIContent(s_unequip);
	gui_armorCon = GUIContent(s_unequip);
	gui_accessCon = GUIContent(s_unequip);
	gui_skillCon = GUIContent("");
	
	if (a_items.Length > 0) {
		a_items[0].setUpItemName();
		currentItem = a_items[0];
	}
	
	//Setup boolean equip
	for (var i : int = 0 ; i < a_equipBoolean.length; i++) {
		a_equipBoolean[i] = false;
	}
}

// Update is called once per frame
public function Update () : void {
	//When the user press M key show the menu window
	if (Input.GetKey(KeyCode.M)) {
		if (b_openMenu == false) {
			b_openMenu = true;
		}
	}
}

//All GUI Class will create in this function
public function OnGUI () : void {
	GUI.skin = customSkin; //Assign our MenuSkin to the Gui Skin
	if (b_openMenu) {	//If open menu == true create a menu window
		r_window = GUI.Window (0, r_window, DoMyWindow, ""); //create a new window by the size of rect
		//This whole code is to amke sure that our window can't be drag outside of the screen area
		///////////////////////////////////////////////////////////////////////////////
		r_window.x = Mathf.Clamp(r_window.x, 0.0, Screen.width - r_window.width);
		r_window.y = Mathf.Clamp(r_window.y, 0.0, Screen.height - r_window.height);
		////////////////////////////////////////////////////////////////////////////////
	}
}

//Our window function operate here
private function DoMyWindow (windowID : int) : void {
	//We create tab button here.
	in_toolbar = GUI.Toolbar (r_tabButton, in_toolbar, s_toolbars, GUI.skin.GetStyle("Tab Button"));
	
	switch (in_toolbar) {
		case 0 : //Status
			//Create a status tab
			StatusWindow();
			break;
		case 1 : //Items
			//Create an item tab
			ItemWindow();
			break;
		case 2 : //Equip
			//Create an equipment tab
			EquipWindow();
			break;
	}
	//Draw our background character texture
	GUI.DrawTexture(r_hero, t_hero);
	
	//We create a close button here
	if (GUI.Button (r_closeBtn, "", GUI.skin.GetStyle("Exit Button"))) {
		b_openMenu = false;
	}
	
	//Make our window dragable in whole area
	GUI.DragWindow();
}

private function StatusWindow() : void {
	GUI.Box (r_statBox, "");
	GUI.Box (r_weaponBox, "");
	GUI.DrawTexture(r_statTexture1, t_statusBox1);
	GUI.DrawTexture(r_statTexture2, t_statusBox2);
	GUI.DrawTexture(r_skillBox, t_skillBox);
	
	CheckMax();
	
	GUI.Label(r_hpLabel, currentHP.ToString() + "/" + fullHP.ToString(), "Text Amount");
	GUI.Label(r_mpLabel, currentMP.ToString() + "/" + fullMP.ToString(), "Text Amount");
	GUI.Label(r_lvLabel, currentLV.ToString(), "Text Amount");
	GUI.Label(r_expLabel, currentEXP.ToString(), "Text Amount");
	GUI.Label(r_nextLabel, currentNEXT.ToString(), "Text Amount");
	
	GUI.Label(r_atkLabel, currentATK.ToString(), "Text Amount");
	GUI.Label(r_defLabel, currentDEF.ToString(), "Text Amount");
	GUI.Label(r_agiLabel, currentAGI.ToString(), "Text Amount");
	GUI.Label(r_intLabel, currentINT.ToString(), "Text Amount");
	GUI.Label(r_lucLabel, currentLUC.ToString(), "Text Amount");
	
	GUI.Label(r_weaponLabel, gui_weaponCon, "Text Item");
	GUI.Label(r_armorLabel, gui_armorCon, "Text Item");
	GUI.Label(r_accessLabel, gui_accessCon, "Text Item");
	GUI.Label(r_skillTexture, gui_skillCon, "Text Item");
}

private function ItemWindow() : void {
	var in_items : int = 8;
	 //Create Item Information box
	GUI.Box (r_itemsBox, "");
	GUI.Box (r_tipBox, "");
    scrollPosition = GUI.BeginScrollView (new Rect (257, 87, 320, 200), scrollPosition, new Rect (0, 0, 280, 40*in_items));
    // We just add a single label to go inside the scroll view. Note how the
    // scrollbars will work correctly with wordwrap.
    var itemsContent : GUIContent[] = new GUIContent[in_items];
    //We create a GUIContent array of key item here (if you have more than 1 items, you can also use your item array instead of the current item)
    for (var i: int = 0; i < in_items; i++) {
    	if (a_items.Length > 0) {
	   	 	if (i == 0) {
	    		itemsContent[i] = GUIContent(currentItem.itemNA, currentItem.icon, "Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat.");
	    	} else {
	    		itemsContent[i] = GUIContent(currentItem.itemNA, currentItem.icon, "This is key " + i);
	    	}
    	} else {
    		itemsContent[i] = GUIContent("NONE", "");
    	}
    }
    
	//We create grid button here.
	in_toolItems = GUI.SelectionGrid (Rect (0, 0, 280, 40*in_items), in_toolItems, itemsContent, 1, GUI.skin.GetStyle("Selected Item"));
    GUI.EndScrollView (); //End Scroll Area
    
	//Checking if there is an item information
	var s_info : String = itemsContent[in_toolItems].tooltip;
	if (s_info == "") {
		s_info = "Show items information here";
	}
	var style : GUIStyle = GUI.skin.GetStyle("Label");
	if (GUI.tooltip != "") {
		//Get height from this style
		var f_height : float = style.CalcHeight(GUIContent(GUI.tooltip), 330.0);
		scrollPosition2 = GUI.BeginScrollView (new Rect (257, 343, 320, 75), scrollPosition2, new Rect (0, 0, 280, f_height));	
		GUI.Label(new Rect (0, 0, 280, f_height), GUI.tooltip);
	} else {
		//Get height from this style
		f_height = style.CalcHeight(GUIContent(s_info), 330.0);
		scrollPosition2 = GUI.BeginScrollView (new Rect (257, 343, 320, 75), scrollPosition2, new Rect (0, 0, 280, f_height));	
		GUI.Label(new Rect (0, 0, 280, f_height), s_info);
	}
	GUI.EndScrollView ();
}

private function EquipWindow() : void {
	GUI.Box (r_equipBox, "");
	GUI.Box (r_equipWeaponBox, "");
	GUI.DrawTexture(r_statTextureEquip, t_statusBox2);
	GUI.DrawTexture(r_skillBoxEquip, t_skillBox);
	
	SetupEquipBox();
}

//Setting the ability to enabled or disable the button
private function SetupEquipBox () : void {
	var equipContent : GUIContent[] = [gui_weaponCon, gui_armorCon, gui_accessCon, gui_skillCon];
	for (var i : int = 0; i < a_equipBoolean.length; i++) {
		if (a_equipBoolean[i] == true) {
			//Set up disabled Button
    		GUI.Label(r_equipRect[i], equipContent[i], "Disabled Click");
    		//Show each equipment window
    		switch (i) {
				case 0:
					ShowWeapon();
					break;
				case 1:
					ShowArmor();
					break;
				case 2:
					ShowAccess();
					break;
				case 3:
					ShowSkill();
					break;
			}
		} else {
    		//Set up enabled Button
    		if (GUI.Button(r_equipRect[i], equipContent[i], "Selected Item")) {
			    a_equipBoolean[i] = true;
			    //Set others to false
			    for (var j : int = 0; j < a_equipBoolean.length; j++) {
			    	if (i != j) {
			    		a_equipBoolean[j] = false;
			    	}
			    }
			}
		}
	}
}

private function ShowWeapon () : void {
	var in_items : int = 6;
	var itemsContent : GUIContent[] = new GUIContent[in_items];
    //We create a GUIContent array of key item here (if you have more than 1 items, you can also use your item array instead of the current item)
    for (var i: int = 0; i < in_items; i++) {
   	 	if (i == 0) {
    		itemsContent[i] = GUIContent(s_unequip, "");
    	} else {
    		itemsContent[i] = GUIContent(a_weapons[0].name, a_weapons[0].icon);
    	}
    }
	scrollPosition3 = GUI.BeginScrollView (new Rect (257, 300, 320, 120), scrollPosition3, new Rect (0, 0, 280, 40*in_items));
	//We create grid button here.
	in_toolWeapons = GUI.SelectionGrid (Rect (0, 0, 280, 40*in_items), in_toolWeapons, itemsContent, 1, GUI.skin.GetStyle("Selected Item"));
    //End the scrollview we began above.
    GUI.EndScrollView ();
    
    gui_weaponCon = itemsContent[in_toolWeapons];
}

private function ShowArmor () : void {
	var in_items : int = 6;
	var itemsContent : GUIContent[] = new GUIContent[in_items];
    //We create a GUIContent array of key item here (if you have more than 1 items, you can also use your item array instead of the current item)
    for (var i: int = 0; i < in_items; i++) {
   	 	if (i == 0) {
    		itemsContent[i] = GUIContent(s_unequip, "");
    	} else {
    		itemsContent[i] = GUIContent(a_armors[0].name, a_armors[0].icon);
    	}
    }
	scrollPosition3 = GUI.BeginScrollView (new Rect (257, 300, 320, 120), scrollPosition3, new Rect (0, 0, 280, 40*in_items));
	//We create grid button here.
	in_toolArmors = GUI.SelectionGrid (Rect (0, 0, 280, 40*in_items), in_toolArmors, itemsContent, 1, GUI.skin.GetStyle("Selected Item"));
    // End the scrollview we began above.
    GUI.EndScrollView ();
    
    gui_armorCon = itemsContent[in_toolArmors];
}

private function ShowAccess () : void {
	var in_items : int = 6;
	var itemsContent : GUIContent[] = new GUIContent[in_items];
	//We create a GUIContent array of key item here (if you have more than 1 items, you can also use your item array instead of the current item)
    for (var i: int = 0; i < in_items; i++) {
   	 	if (i == 0) {
    		itemsContent[i] = GUIContent(s_unequip, "");
    	} else {
    		itemsContent[i] = GUIContent(a_accessories[0].name, a_accessories[0].icon);
    	}
    }
	scrollPosition3 = GUI.BeginScrollView (new Rect (257, 300, 320, 120), scrollPosition3, new Rect (0, 0, 280, 40*in_items));
	//We create grid button here.
	in_toolAccess = GUI.SelectionGrid (Rect (0, 0, 280, 40*in_items), in_toolAccess, itemsContent, 1, GUI.skin.GetStyle("Selected Item"));
    // End the scrollview we began above.
    GUI.EndScrollView ();
    
    gui_accessCon = itemsContent[in_toolAccess];
}

private function ShowSkill () : void {
	var in_items : int = a_skills.length + 1;
	var itemsContent : GUIContent[] = new GUIContent[in_items];
	//We create a GUIContent array of key item here (if you have more than 1 items, you can also use your item array instead of the current item)
    for (var i: int = 0; i < in_items; i++) {
   	 	if (i == 0) {
    		itemsContent[i] = GUIContent(t_skillBox);
    	} else {
    		itemsContent[i] = GUIContent(a_skills[i-1]);
    	}
    }
	scrollPosition3 = GUI.BeginScrollView (new Rect (253, 286, 330, 140), scrollPosition3, new Rect (0, 0, 600, 117));
	//We create grid button here.
	in_toolskill = GUI.SelectionGrid (Rect (0, 4, 600, 117), in_toolskill, itemsContent, in_items, GUI.skin.GetStyle("Selected Item"));
    // End the scrollview we began above.
    GUI.EndScrollView ();
    if(in_toolskill != 0) {
   		gui_skillCon = itemsContent[in_toolskill];	
    } else {
    	gui_skillCon = GUIContent("");
    }
}

private function CheckMax () : void {
	fullHP = Mathf.Clamp(fullHP, 0.0, maxHP);
	fullMP = Mathf.Clamp(fullMP, 0.0, maxMP);
	currentHP = Mathf.Clamp(currentHP, 0.0, fullHP);
	currentMP = Mathf.Clamp(currentMP, 0.0, fullMP);
	currentLV = Mathf.Clamp(currentLV, 0.0, maxLV);
	currentEXP = Mathf.Clamp(currentEXP, 0.0, maxEXP);
	currentNEXT = Mathf.Clamp(currentNEXT, 0.0, maxNEXT);
	currentATK = Mathf.Clamp(currentATK, 0.0, maxATK);
	currentDEF = Mathf.Clamp(currentDEF, 0.0, maxDEF);
	currentAGI = Mathf.Clamp(currentAGI, 0.0, maxAGI);
	currentINT = Mathf.Clamp(currentINT, 0.0, maxINT);
	currentLUC = Mathf.Clamp(currentLUC, 0.0, maxLUC);
}