package fb2g05;
import java.awt.Color;
import java.awt.geom.Point2D;
import robocode.AdvancedRobot;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;
public class C0119214 extends AdvancedRobot {
	static Point2D.Double [] EnemyPoint = new Point2D.Double [100];
	int count = 0;
	double oldEnemyHeading;
	long Time;
	double Energy;
	double X;
	double Y;
	double Heading;
	double RadarHeading;
	double RadarTurnRemaining;
	double GunHeading;
	C0119214.Enemy enemy;
	C0119214.Enemy LastEnemy;
	boolean sentry = true;
	@Override
	public void run() {
		this.RoboSet();
		while (true) {
			this.UpdateMyInfo();
			this.Radar();
			this.execute();
		}
	}
	void RoboSet() {
		this.setBodyColor(Color.BLACK);
		this.setGunColor(Color.RED);
		this.setRadarColor(Color.RED);
		this.setBulletColor(Color.RED);
		this.setScanColor(Color.RED);
		this.setAdjustGunForRobotTurn(true);
		this.setAdjustRadarForGunTurn(true);
	}
	void UpdateMyInfo() {
		this.Time = this.getTime();
		this.Energy = this.getEnergy();
		this.X = this.getX();
		this.Y = this.getY();
		this.Heading = this.getHeading();
		this.RadarHeading = this.getRadarHeading();
		this.RadarTurnRemaining = this.getRadarTurnRemaining();
		this.GunHeading = this.getGunHeading();
		if (this.enemy != null) {
			this.enemy.AbsoluteBearing = this.Heading + this.enemy.RelativeBearing;
		}
	}
	void Radar() {
		if (this.FullScanNeed()) {
			this.FullScan();
		}
		else {
			this.LockOn();
		}
	}
	boolean FullScanNeed() {
		if (this.enemy == null) {
			return true;
		}
		else {
			return (this.Time - this.enemy.TimeScanned > 3);
		}
	}
	void FullScan() {
		if (Math.abs(this.RadarTurnRemaining) < 45) {
			double Center = this.CulcHeading(this.X, this.Y, this.getBattleFieldWidth() / 2, this.getBattleFieldHeight() / 2);
		    double Or = Math.signum(Utils.normalRelativeAngleDegrees(Center - this.GunHeading));
		    double Turn = Or * 360;
		    this.setTurnRight(Turn);
		    this.setTurnGunRight(Turn);
		    this.setTurnRadarRight(Turn);
		}
	}
	protected double CulcHeading(double x1, double y1, double x2, double y2) {
		double x = x2 - x1;
		double y = y2 - y1;
		double Angle = Math.toDegrees(Math.atan2(y, x));
		return Utils.normalAbsoluteAngleDegrees(90 - Angle);
	}
	void LockOn() {
		if(this.enemy.Distance < 1000 && this.sentry == false && this.getOthers() == 1 && this.enemy.Energy < this.Energy - 10) {
			double RadarOff = Utils.normalRelativeAngleDegrees(this.enemy.AbsoluteBearing - this.RadarHeading);
			double AddTurn = Math.signum(RadarOff) * Math.min(Math.toDegrees(Math.atan(26 / this.enemy.Distance)), 45);
			double RadarTurn = RadarOff + AddTurn;
			this.setTurnRadarRight(RadarTurn);
		}
	}
	@Override
	public void onScannedRobot(ScannedRobotEvent event) {
		double MyX = this.getX();
		double MyY = this.getY();
		double MyHeadingRad = this.getHeadingRadians();
		double EnemyDistance =event.getDistance();
		double EnemyBearingRad = event.getBearingRadians();
		double AbsEnemyBearingRad = MyHeadingRad + EnemyBearingRad;


		if(event.isSentryRobot()) {
			this.sentry = true;
		}
		else {
			this.sentry = false;
		}
		//自機の座標から敵の座標を計算
		double EnemyX = MyX + EnemyDistance * Math.sin(AbsEnemyBearingRad);
		double EnemyY = MyY + EnemyDistance * Math.cos(AbsEnemyBearingRad);


		//敵の座標をEnemyPointに記録
		EnemyPoint[count] = new Point2D.Double(EnemyX, EnemyY);
		//台数を記録、及びリセット
		if (++ count >= getOthers()) {
			count = 0;
		}
		//AntiGravityCalculator
		double XForce = 0;
		double YForce = 0;
		for (int i = 0; i < getOthers() && EnemyPoint[i] != null; i++) {
			//直交座標から極座標へ変換しシータを返す
			double Bearing = Utils.normalAbsoluteAngle(Math.atan2(EnemyPoint[i].x - MyX, EnemyPoint[i].y - MyY));
			//距離を計算
			double Distance = EnemyPoint[i].distance(MyX, MyY);
			//反重力を計算
			double EnemyForce = 0;
			if(this.sentry == true) {
				EnemyForce = 10;
				XForce -= EnemyForce * Math.sin(Bearing) / Distance / Distance;
				YForce -= EnemyForce * Math.cos(Bearing) / Distance / Distance;
			}
			else {
				EnemyForce = 1;
				XForce -= EnemyForce * Math.sin(Bearing) / Distance / Distance;
				YForce -= EnemyForce * Math.cos(Bearing) / Distance / Distance;


				if(this.getOthers() == 1 && this.enemy.Energy < this.Energy - 10) {
					EnemyForce = Double.POSITIVE_INFINITY;
					XForce += EnemyForce * Math.sin(Bearing) / Distance / Distance;
					YForce += EnemyForce * Math.cos(Bearing) / Distance / Distance;
					this.setTurnGunLeft(this.getHeading() - this.getGunHeading());
					this.setFire(0.1);
				}
			}
		}
		final double WallForce = 50;
		XForce -= WallForce / Math.pow(Point2D.distance(MyX, MyY, this.getBattleFieldWidth(), MyY), 3);
		XForce += WallForce / Math.pow(Point2D.distance(MyX, MyY, 0, MyY), 3);
		YForce -= WallForce / Math.pow(Point2D.distance(MyX, MyY, MyX, this.getBattleFieldHeight()), 3);
		YForce += WallForce / Math.pow(Point2D.distance(MyX, MyY, MyX, 0), 3);


		//直交座標から極座標へ変換しそのシータを返す
		double Angle = Math.atan2(XForce, YForce);


		//移動
		if (XForce == 0 && YForce == 0) {
			//何もしない
		}
		else if (Math.abs(Angle - MyHeadingRad) < Math.PI / 2) {
			this.setTurnRightRadians(Utils.normalRelativeAngle(Angle - MyHeadingRad));
			this.setAhead(Double.POSITIVE_INFINITY);
		}
		else {
			this.setTurnRightRadians(Utils.normalRelativeAngle(Angle + Math.PI - MyHeadingRad));
			this.setAhead(Double.NEGATIVE_INFINITY);
		}
		this.LastEnemy = this.enemy;
		this.enemy = new Enemy();
		this.enemy.TimeScanned = this.getTime();
		this.enemy.Energy = event.getEnergy();
		this.enemy.RelativeBearing = event.getBearing();
		this.enemy.AbsoluteBearing = Utils.normalAbsoluteAngleDegrees(this.Heading + this.enemy.RelativeBearing);
		this.enemy.Distance = EnemyDistance;
		this.enemy.X = MyX + EnemyDistance * Math.sin(AbsEnemyBearingRad);
		this.enemy.Y = MyY + EnemyDistance * Math.cos(AbsEnemyBearingRad);
		this.enemy.Heading = event.getHeading();
		this.enemy.Velocity = event.getVelocity();
		if (this.LastEnemy != null) {
			this.enemy.Acceleration = (this.enemy.Velocity - this.LastEnemy.Velocity) / (double)(this.enemy.TimeScanned - this.LastEnemy.TimeScanned);
			this.enemy.TurnRate = Utils.normalRelativeAngleDegrees(this.enemy.Heading - this.LastEnemy.Heading) / (double)(this.enemy.TimeScanned - this.LastEnemy.TimeScanned);
		}
	}
	static class Enemy {
		   public long TimeScanned;
		   public double Energy;
		   public double X;
		   public double Y;
		   public double Distance;
		   public double RelativeBearing;
		   public double AbsoluteBearing;
		   public double Heading;
		   public double Velocity;
		   public double TurnRate;
		   public double Acceleration;
		}
}
