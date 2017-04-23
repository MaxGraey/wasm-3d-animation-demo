import {Vec3} from './vec3';

export class Quaternion {
    constructor(
        public x: number = 0,
        public y: number = 0,
        public z: number = 0,
        public w: number = 1
    ) {
    }

    public static fromAxisAngle(axis: Vec3, angle: number): Quaternion {
        let cos2 = Math.cos(angle * 0.5);
        let sin2 = Math.sin(angle * 0.5);

        return new Quaternion(
            sin2 * axis.x(),
            sin2 * axis.y(),
            sin2 * axis.z(),
            cos2
        ).setNormal();
    }

    public setNormal(): Quaternion {
        let x = this.x;
        let y = this.y;
        let z = this.z;
        let w = this.w;

        let mag = 1 / Math.sqrt(x * x + y * y + z * z + w * w);
        this.x *= mag;
        this.y *= mag;
        this.z *= mag;
        this.w *= mag;
        return this;
    }

    public normal(): Quaternion {
        return this.clone().setNormal();
    }

    public clone(): Quaternion {
        return new Quaternion(this.x, this.y, this.z, this.w);
    }

    /**
     * Perform a spherical interpolation between two quaternions
     * https://github.com/toji/gl-matrix/blob/master/src/gl-matrix/quat.js
     * @param {Quaternion} a
     * @param {Quaternion} b
     * @param {number} t between 0 and 1
     */
    public static slerp(a: Quaternion, b: Quaternion, t: number): Quaternion {
        let ax = a.x, ay = a.y, az = a.z, aw = a.w;
        let bx = b.x, by = b.y, bz = b.z, bw = b.w;

        let omega, cosom, sinom, scale0, scale1;
        // calc cosine
        cosom = ax * bx + ay * by + az * bz + aw * bw;
        // adjust signs (if necessary)
        if ( cosom < 0.0 ) {
            cosom = -cosom;
            bx = - bx;
            by = - by;
            bz = - bz;
            bw = - bw;
        }
        // calculate coefficients
        if ((1.0 - cosom) > 0.000001) {
            // standard case (slerp)
            omega  = Math.acos(cosom);
            sinom  = Math.sin(omega);

            let ot = t * omega;
            scale0 = Math.sin(omega - ot) / sinom;
            scale1 = Math.sin(ot) / sinom;
        } else {
            // "from" and "to" quaternions are very close
            //  ... so we can do a linear interpolation
            scale0 = 1.0 - t;
            scale1 = t;
        }

        // calculate final values
        return new Quaternion(
            scale0 * ax + scale1 * bx,
            scale0 * ay + scale1 * by,
            scale0 * az + scale1 * bz,
            scale0 * aw + scale1 * bw
        );
    }

    /**
     * Also taken from the gl-matrix library
     * https://github.com/toji/gl-matrix/blob/master/src/gl-matrix/quat.js
     */
    public static multiply(a: Quaternion, b: Quaternion): Quaternion {
        let ax = a.x, ay = a.y, az = a.z, aw = a.w;
        let bx = b.x, by = b.y, bz = b.z, bw = b.w;
        return new Quaternion(
            ax * bw + aw * bx + ay * bz - az * by,
            ay * bw + aw * by + az * bx - ax * bz,
            az * bw + aw * bz + ax * by - ay * bx,
            aw * bw - ax * bx - ay * by - az * bz
        );
    }

    public static rotateVec3(q: Quaternion, v: Vec3) {
        let u = new Vec3(q.x, q.y, q.z);
        let s = q.w;

        return u.scale(2 * Vec3.dot(u, v))
            .setAdd(v.scale(s * s - Vec3.dot(u, u)))
            .setAdd(Vec3.cross(u, v).setScale(2 * s));
    }

    public static IDENTITY = new Quaternion();
}
