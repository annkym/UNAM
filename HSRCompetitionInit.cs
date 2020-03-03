using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using SIGVerse.ToyotaHSR;
using SIGVerse.Common;
using SIGVerse.Competition.PonNet;
using UnityEngine.EventSystems;
using SIGVerse.Competition.Handyman;
using System.Threading;
using UnityEngine.UI;
using UnityEngine.SceneManagement;


namespace SIGVerse.Competition
{
	public enum PlacingStep
	{
		Initialize,
		WaitForIamReady,
		SetUpScene,
		WaitForImagesCaptured,
		ExcecutePlacing,
		CalculateScore,
		WaitForNextTask,
		Stoping,
		Final,
	}

	public class HSRCompetitionInit : MonoBehaviour, IRosMsgReceiveHandler
	{

		private const string FileName = "/../SIGVerseConfig/TeamLogo.jpg";

		private string teamLogoPath;
		public MeshRenderer teamLogoRenderer;

		private PlacingStep step;
		private static PlacingTools tool;

		public int totScenes = 10;		//Total scenes to be collected
		public int numOfScenes = 1;		//Number of scenes per location
		public int maxObj = 25;

		public int numStrategies = 4;	//Total number of placing strategies

		//For different locations
		public GameObject ponNetLayout;
		public static int sceneNumber = 0;	//Scene counter

		public bool scrambled = false;		//To set obstacles scattered or not

		private const string MsgAreYouReady = "Are_you_ready?";
		private const string MsgSettingCompleted = "Setting_completed";
		private const string MsgScore = "Score";
		private const string MsgStopProcess = "Stop_process";


		private const string MsgIamReady = "I_am_ready";
		private const string MsgImagesCaptured = "Images_captured";
		private const string MsgObjectPlaced = "Object_placed";
		private const string MsgScoreSaved = "Score_saved";
		private const string MsgProcessStopped = "Process_stopped";

		private const int SendingInterval = 100;
		private bool isAllTaskFinished;

		private int fakeCount;
		private String scorePlacing;

		public PonNetScoreManager scoreMgr;

		public GameObject scorePanel;

		private Text scoreValText;
		private Text sceneValText;

		public ScenePonNet scenes;

		public GameObject test;

		private int strategy;

		void Awake ()
		{			
			tool = new PlacingTools ();
			this.teamLogoPath = Application.dataPath + FileName;
			this.scoreValText = this.scorePanel.transform.Find("ScoreValText").GetComponent<Text>();
			this.sceneValText = this.scorePanel.transform.Find("TotalValText").GetComponent<Text>();

		}

		// Initialization
		void Start ()
		{
			//SIGVerseLogger.Info ("*********************************************** STARTING PLACING DATA COLLECTION **********************************************");
			Texture texture = this.CreateTextureFromImageFile (teamLogoPath);

			if (texture != null) {
				this.teamLogoRenderer.material.mainTexture = texture;
			} else {
				this.teamLogoRenderer.gameObject.SetActive (false);
			}
			this.step = PlacingStep.Initialize;
			this.isAllTaskFinished = false;
			this.fakeCount = 0;
			this.scorePlacing = "";
			if (sceneNumber >= totScenes) {
				this.step = PlacingStep.Final;
			}

			Vector3 size = test.GetComponent<Renderer> ().bounds.size;
			SIGVerseLogger.Info ("Objeto " + test.name + "Size x = " + size.x + " y =" + size.y + " z = " + size.z);
		}



		// Update is called once per frame
		void Update ()
		{
			try {

				if (this.isAllTaskFinished) {
					return;
				}

				switch (this.step) {

				case PlacingStep.Initialize:
					//Configure everything just the first time
					if (tool.IsConnectedToRos ()) { 
						this.step++;
					}
					break;

				case PlacingStep.WaitForIamReady:
					//Send Are you ready? message and wait for answer
					this.SendRosMessage (MsgAreYouReady, string.Empty);
					break;
				
				case PlacingStep.SetUpScene:
					//Cycle goes back here to start new scene
					this.scorePlacing = "";
					if (this.fakeCount < numOfScenes) {
						//Scene configuration and showing
						this.setUpScene();
						this.fakeCount++;
						this.step++;
						sceneNumber++;
						this.scoreValText.text = "XXX";
						this.sceneValText.text = sceneNumber.ToString();
						this.strategy = UnityEngine.Random.Range (1,numStrategies+1);
					} else {						
						//When finishing the number of scenes configured goes to last step
						this.step = PlacingStep.Stoping;
					}
					break;	
				
				case PlacingStep.WaitForImagesCaptured:
					string cameraHeigth;
					if(PlacingTools.typeLower){
						cameraHeigth = "L-";
					}else{
						cameraHeigth = "U-";
					}
					//Send Setting completed message and wait for Images captured message from robot
					//Sending camera height for metadata file
					cameraHeigth += tool.getFurnitureID(); 
					//Sending strategy number from 1 to max strategies number
					cameraHeigth += "-" + this.strategy;
					this.SendRosMessage (MsgSettingCompleted, cameraHeigth);
					break;
				//While robot is placing object on table, unity starts scoring manager for scene
				case PlacingStep.ExcecutePlacing:
					tool.startPlacing();
					break;		
				//When receiving Object Placed message calculates score
				case PlacingStep.CalculateScore:
					this.scorePlacing = tool.finishPlacing();
					this.step++;
					break;
				//Send score and wait for Score Saved message, finally goes back to WaitForIamReady
				case PlacingStep.WaitForNextTask:
					string sc = this.scorePlacing;
					this.scoreValText.text = this.scorePlacing.Split(':')[0];
					sc += "-" + tool.getTargetID();
					this.SendRosMessage (MsgScore, sc);
					tool.resetRobot ();
					break;
				
				//When finished all scenes send Stop Process message
				case PlacingStep.Stoping:
					this.scenes.moveHouse ();
					this.scoreValText.text = "XXX";
					this.SendRosMessage (MsgStopProcess, string.Empty);
					this.isAllTaskFinished = true;
					Start();
					break;

				case PlacingStep.Final:
					this.sceneValText.text = "FINISHED";
					this.SendRosMessage (MsgStopProcess, string.Empty);
					this.isAllTaskFinished = true;
					break;
				}

			} catch (Exception exception) {
				SIGVerseLogger.Error (exception.Message);
				SIGVerseLogger.Error (exception.StackTrace);
				this.ApplicationQuitAfter1sec ();
			}
		}

		private Texture CreateTextureFromImageFile (string path)
		{
			try {
				byte[] imageBinary = this.ReadImageFile (path);
				Texture2D texture = new Texture2D (1, 1);
				texture.LoadImage (imageBinary);
				return texture;
			} catch (Exception exception) {
				SIGVerse.Common.SIGVerseLogger.Error ("Couldn't open TeamLogo.jpg. msg=" + exception.Message);
				return null;
			}
		}

		private byte[] ReadImageFile (string path)
		{
			FileStream fileStream = new FileStream (path, FileMode.Open, FileAccess.Read);
			BinaryReader binaryReader = new BinaryReader (fileStream);
			byte[] values = binaryReader.ReadBytes ((int)binaryReader.BaseStream.Length);

			binaryReader.Close ();
			fileStream.Close ();

			return values;
		}

		//To send message to ros
		private void SendRosMessage (string message, string detail)
		{
			ExecuteEvents.Execute<IRosMsgSendHandler>
			(
				target: this.gameObject, 
				eventData: null, 
				functor: (reciever, eventData) => reciever.OnSendRosMessage (message, detail)
			);
		}

		//To receive message from ros
		public void OnReceiveRosMessage (RosBridge.handyman.HandymanMsg handymanMsg)
		{
			//SIGVerseLogger.Info ("MESSAGE RECEIVED" + handymanMsg.message);
			if (handymanMsg.message == MsgIamReady) {				
				this.step = PlacingStep.SetUpScene;
			}
			if (handymanMsg.message == MsgImagesCaptured) {
				tool.setTargetObject();
				this.step = PlacingStep.ExcecutePlacing;
			}
			if (handymanMsg.message == MsgObjectPlaced) {
				this.step = PlacingStep.CalculateScore;
			}
			if (handymanMsg.message == MsgScoreSaved) {
				tool.disapearTarget();
				this.step = PlacingStep.WaitForIamReady;

			}
			//Debug.Log ("Change " + this.step);
		}

		private void ApplicationQuitAfter1sec ()
		{
			Thread.Sleep (1000);
			Application.Quit ();
		}

		private void setUpScene()
		{			
			tool.resetRobot ();
			tool.setFurniture (scrambled);
			if (scrambled) {
				tool.setSceneObjects (maxObj);
			} else {
				tool.setStandUpObjects(maxObj);
			}


		}

	}
}
