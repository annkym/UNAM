using UnityEngine;
using UnityEngine.UI;
using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using SIGVerse.Common;
using SIGVerse.ToyotaHSR;
using UnityEngine.EventSystems;
using SIGVerse.Competition.Handyman;

namespace SIGVerse.Competition.PonNet
{
	public class PonNetScoreManager : MonoBehaviour, ITransferredCollisionHandler
	{
		
		private int score = 0;
		public static bool recordScore;
		private float scoreThreshold = 0.1f;
		private const string TagGraspingCandidates = "GraspingCandidates";
		private const string TagObstacleCandidates = "ObstacleCandidates";
		private static int collisionO;
		private static int collisionT;
		private static int collisionA;
		private static int collisionF;

		private static float maxVel;



		public static void startRecordingScore(){			
			recordScore = true;
		}

		public String getScore(){	
			/*SIGVerseLogger.Info ("************************************************************************************************ COLLISIONS O ************"+ collisionO  );
			SIGVerseLogger.Info ("************************************************************************************************ COLLISIONS T ************"+ collisionT  );
			SIGVerseLogger.Info ("************************************************************************************************ COLLISIONS A ************"+ collisionA  );
			SIGVerseLogger.Info ("************************************************************************************************ COLLISIONS F ************"+ collisionF  );
			*/
			if (collisionO != 0 || collisionT != 0 || collisionA != 0 || collisionF != 0) {
				score = 0;
			} 
			String scoreText = score + ":" + collisionO + ":" + collisionT + ":" + collisionA + ":" + collisionF + "-" + maxVel; 
			Debug.Log ("MAXIMA VEL " + maxVel);
			return scoreText;
		}

		public void stopRecordingScore(){			
			recordScore = false;
			collisionO = 0;
			collisionT = 0;
			collisionA = 0;
			collisionF = 0;
			this.score = 100;
			maxVel = 0.0f;
		}
			
		void Start()
		{
			stopRecordingScore ();
		}


		public void OnTransferredCollisionEnter(SIGVerse.Competition.CollisionType collisionType, Collision collision, float collisionVelocity, float effectScale)
		{			

		}

		public void scoreCollision(GameObject obj, Collision collision){
			if (recordScore && collision.relativeVelocity.magnitude > scoreThreshold) {
				//Arm-Obstacle
				String colPathArm = SIGVerseUtils.GetHierarchyPath (collision.collider.transform);
				bool colArm = colPathArm.Contains ("arm_lift_link");
				if(colArm){
					//SIGVerseLogger.Info ("************************************************************************************************ ARM COLLISION  ************"  );
					collisionA++;
				}
				//Obstacle-Obstacle
				bool colObs = collision.gameObject.CompareTag(TagObstacleCandidates);
				if(colObs){
					//SIGVerseLogger.Info ("************************************************************************************************ OBSTACLE COLLISION  ************"  );
					collisionO++;
				}

				//Target-Obstacle
				bool colTar = collision.gameObject.CompareTag(TagGraspingCandidates);
				if(colTar){
					//SIGVerseLogger.Info ("************************************************************************************************ TARGET COLLISION  ************"  );
					collisionT++;
				}
				//Furniture-Obstacle
				String colPathFur = SIGVerseUtils.GetHierarchyPath (collision.collider.transform);
				bool colFur = colPathFur.Contains ("Tables");
				if(colFur){
					//SIGVerseLogger.Info ("************************************************************************************************ FURNITURE COLLISION  ************"  );
					collisionF++;
				}

				if (collision.relativeVelocity.magnitude > maxVel) {
					maxVel = collision.relativeVelocity.magnitude;
				}

			}
			
		}

	}

}

