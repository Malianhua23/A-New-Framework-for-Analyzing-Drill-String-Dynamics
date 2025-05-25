#pragma once
// contactmodelTBeam.h

#include "contactmodel/src/contactmodelmechanical.h"

#ifdef TBeam_LIB
#  define TBeam_EXPORT EXPORT_TAG
#elif defined(NO_MODEL_IMPORT)
#  define TBeam_EXPORT
#else
#  define TBeam_EXPORT IMPORT_TAG
#endif

namespace cmodelsxd {
	using namespace itasca;

	class ContactModelTBeam : public ContactModelMechanical {
	public:
		// Constructor: Set default values for contact model properties.
		TBeam_EXPORT ContactModelTBeam();
		// Destructor, called when contact is deleted: free allocated memory, etc.
		TBeam_EXPORT virtual ~ContactModelTBeam();
		// Contact model name (used as keyword for commands and FISH).
		virtual QString  getName() const { return "tbeam"; }
		// The index provides a quick way to determine the type of contact model.
		// Each type of contact model in PFC must have a unique index; this is assigned
		// by PFC when the contact model is loaded. This index should be set to -1.
		virtual void     setIndex(int i) { index_ = i; }
		virtual int      getIndex() const { return index_; }
		// Contact model version number (e.g., MyModel05_1). The version number can be
		// accessed during the save-restore operation (within the archive method,
		// testing {stream.getRestoreVersion() == getMinorVersion()} to allow for 
		// future modifications to the contact model data structure.
		virtual uint     getMinorVersion() const;
		// Copy the state information to a newly created contact model.
		// Provide access to state information, for use by copy method.
		virtual void     copy(const ContactModel *c);
		// Provide save-restore capability for the state information.
		virtual void     archive(ArchiveStream &);
		// Enumerator for the properties.
		enum PropertyKeys {
			kwE = 1
			, kwNu
			, kwBHgiven
			, kwA
			, kwJ
			, kwIy
			, kwIz
			, kwb
			, kwh
			, kwYdir
			, kwL
			, kwKn
			, kwKs
			, kwYaxis
			, kwForce
			, kwMoment
			, kwEstr
			, kwUserArea
			, kwRGap
		};
		// Contact model property names in a comma separated list. The order corresponds with
		// the order of the PropertyKeys enumerator above. One can visualize any of these 
		// properties in PFC automatically. 
		virtual QString  getProperties() const {
			return  "tbm_E"
				",tbm_nu"
				",tbm_bhGiven"
				",tbm_A"
				",tbm_J"
				",tbm_Iy"
				",tbm_Iz"
				",tbm_b"
				",tbm_h"
				",tbm_Ydir"
				",tbm_L"
				",tbm_kn"
				",tbm_ks"
				",tbm_Yaxis"
				",tbm_force"
				",tbm_moment"
				",tbm_Estr"
				",user_area"
				",rgap";
		}

		// Return the specified contact model property.
		virtual QVariant getProperty(uint i, const IContact *) const;
		// The return value denotes whether or not the property corresponds to the global
		// or local coordinate system (TRUE: global system, FALSE: local system). The
		// local system is the contact-plane system (nst) defined as follows.
		// If a vector V is expressed in the local system as (Vn, Vs, Vt), then V is
		// expressed in the global system as {Vn*nc + Vs*sc + Vt*tc} where where nc, sc
		// and tc are unit vectors in directions of the nst axes.
		// This is used when rendering contact model properties that are vectors.
		virtual bool     getPropertyGlobal(uint i) const;
		// Set the specified contact model property, ensuring that it is of the correct type
		// and within the correct range --- if not, then throw an exception.
		// The return value denotes whether or not the update has affected the timestep
		// computation (by having modified the translational or rotational tangent stiffnesses).
		// If true is returned, then the timestep will be recomputed.
		virtual bool     setProperty(uint i, const QVariant &v, IContact *);
		// The return value denotes whether or not the property is read-only
		// (TRUE: read-only, FALSE: read-write).
		virtual bool     getPropertyReadOnly(uint i) const;

		// The return value denotes whether or not the property is inheritable
		// (TRUE: inheritable, FALSE: not inheritable). Inheritance is provided by
		// the endPropertyUpdated method.
		virtual bool     supportsInheritance(uint i) const { return false; }
		// Return whether or not inheritance is enabled for the specified property.
		virtual bool     getInheritance(uint i) const { return false; }
		// Set the inheritance flag for the specified property.
		virtual void     setInheritance(uint i, bool b) { }

		// Prepare for entry into ForceDispLaw. The validate function is called when:
		// (1) the contact is created, (2) a property of the contact that returns a true via
		// the setProperty method has been modified and (3) when a set of cycles is executed
		// via the {cycle N} command.
		// Return value indicates contact activity (TRUE: active, FALSE: inactive).
		virtual bool    validate(ContactModelMechanicalState *state, const double &timestep);
		// The endPropertyUpdated method is called whenever a surface property (with a name
		// that matches an inheritable contact model property name) of one of the contacting
		// pieces is modified. This allows the contact model to update its associated
		// properties. The return value denotes whether or not the update has affected
		// the time step computation (by having modified the translational or rotational
		// tangent stiffnesses). If true is returned, then the time step will be recomputed.  
		virtual bool    endPropertyUpdated(const QString &name, const IContactMechanical *c) { return false; }
		// The forceDisplacementLaw function is called during each cycle. Given the relative
		// motion of the two contacting pieces (via
		//   state->relativeTranslationalIncrement_ (Ddn, Ddss, Ddst)
		//   state->relativeAngularIncrement_       (Dtt, Dtbs, Dtbt)
		//     Ddn  : relative normal-displacement increment, Ddn > 0 is opening
		//     Ddss : relative  shear-displacement increment (s-axis component)
		//     Ddst : relative  shear-displacement increment (t-axis component)
		//     Dtt  : relative twist-rotation increment
		//     Dtbs : relative  bend-rotation increment (s-axis component)
		//     Dtbt : relative  bend-rotation increment (t-axis component)
		//       The relative displacement and rotation increments:
		//         Dd = Ddn*nc + Ddss*sc + Ddst*tc
		//         Dt = Dtt*nc + Dtbs*sc + Dtbt*tc
		//       where nc, sc and tc are unit vectors in direc. of the nst axes, respectively.
		//       [see {Table 1: Contact State Variables} in PFC Model Components:
		//       Contacts and Contact Models: Contact Resolution]
		// ) and the contact properties, this function must update the contact force and
		// moment (via state->force_, state->momentOn1_, state->momentOn2_).
		//   The force_ is acting on piece 2, and is expressed in the local coordinate system
		//   (defined in getPropertyGlobal) such that the first component positive denotes
		//   compression. If we define the moment acting on piece 2 by Mc, and Mc is expressed
		//   in the local coordinate system (defined in getPropertyGlobal), then we must set
		//   {state->momentOn1_ = Mc}, {state->momentOn2_ = Mc} and call
		//   state->getMechanicalContact()->updateResultingTorquesLocal(...) after having set
		//   force_.
		// The return value indicates the contact activity status (TRUE: active, FALSE:
		// inactive) during the next cycle.
		// Additional information:
		//   * If state->activated() is true, then the contact has just become active (it was
		//     inactive during the previous time step).
		//   * Fully elastic behavior is enforced during the SOLVE ELASTIC command by having
		//     the forceDispLaw handle the case of {state->canFail_ == true}.
		virtual bool    forceDisplacementLaw(ContactModelMechanicalState *state, const double &timestep);
		// The getEffectiveXStiffness functions return the translational and rotational
		// tangent stiffnesses used to compute a stable time step. When a contact is sliding,
		// the translational tangent shear stiffness is zero (but this stiffness reduction
		// is typically ignored when computing a stable time step). If the contact model
		// includes a dashpot, then the translational stiffnesses must be increased (see
		// Potyondy (2009)).
		//   [Potyondy, D. 'Stiffness Matrix at a Contact Between Two Clumps,' Itasca
		//   Consulting Group, Inc., Minneapolis, MN, Technical Memorandum ICG6863-L,
		//   December 7, 2009.]
		virtual DVect2  getEffectiveTranslationalStiffness() const;
		virtual DAVect  getEffectiveRotationalStiffness() const;

		// Return a new instance of the contact model. This is used in the CMAT
		// when a new contact is created. 
		virtual ContactModelTBeam *clone() const { return NEWC(ContactModelTBeam()); }
		// The getActivityDistance function is called by the contact-resolution logic when
		// the CMAT is modified. Return value is the activity distance used by the
		// checkActivity function. 
		virtual double getActivityDistance() const { return rgap_; }
		// The isOKToDelete function is called by the contact-resolution logic when...
		// Return value indicates whether or not the contact may be deleted.
		// If TRUE, then the contact may be deleted when it is inactive.
		// If FALSE, then the contact may not be deleted (under any condition).
		virtual bool   isOKToDelete() const { return false; }
		// Zero the forces and moments stored in the contact model. This function is called
		// when the contact becomes inactive.
		virtual void   resetForcesAndMoments();
		virtual void   setForce(const DVect &v, IContact *c);
		virtual void   setArea(const double &d) { userArea_ = d; }
		// The checkActivity function is called by the contact-resolution logic when...
		// Return value indicates contact activity (TRUE: active, FALSE: inactive).
		virtual bool   checkActivity(const double &gap) { return true; }
		const DVect &  F() const { return F_; }
		void           F(const DVect &f) { F_ = f; }
		const DVect &  M() const { return M_; }
		void           M(const DVect &f) { M_ = f; }
		const double & Estr() const { return Estr_; }
		void           Estr(const double &d) { Estr_ = d; }
		const double & kn() const { return kn_; }
		void           kn(const double &d) { kn_ = d; }
		const double & ks() const { return ks_; }
		void           ks(const double &d) { ks_ = d; }
		const double & rgap() const { return rgap_; }
		void           rgap(const double &d) { rgap_ = d; }
		const double & L() const { return L_; }
		void           L(const double &d) { L_ = d; }
		const DVect &  Ydir() const { return Ydir_; }
		void           Ydir(const DVect &f) { Ydir_ = f; }

		const double & h() const { return h_; }
		void           h(const double &d) { h_ = d; }
		const double & b() const { return b_; }
		void           b(const double &d) { b_ = d; }
		const double & Iz() const { return Iz_; }
		void           Iz(const double &d) { Iz_ = d; }
		const double & Iy() const { return Iy_; }
		void           Iy(const double &d) { Iy_ = d; }
		const double & J() const { return J_; }
		void           J(const double &d) { J_ = d; }
		const double & A() const { return A_; }
		void           A(const double &d) { A_ = d; }
		const int & bhGiven() const { return bhGiven_; }
		void           bhGiven(const int &d) { bhGiven_ = d; }
		const double & nu() const { return nu_; }
		void           nu(const double &d) { nu_ = d; }
		const double & E() const { return E_; }
		void           E(const double &d) { E_ = d; }

		virtual DVect    getForce(const IContactMechanical *) const;
		virtual DAVect   getMomentOn1(const IContactMechanical *) const;
		virtual DAVect   getMomentOn2(const IContactMechanical *) const;

	private:
		static int index_;

		// beam model properties:
		double E_;    // Young's modulus
		double nu_;   // Poisson's ratio
		int    bhGiven_; // cross-sectional properties code (0: specify directly, 1: specify b and h)
		double A_;    // cross-sectional area
		double J_;    // polar moment of inertia
		double Iy_;   // moment of inertia about y-axis
		double Iz_;   // moment of inertia about z-axis
		double b_;    // cross-sectional width  aligned with y-axis (bhGiven == 1)
		double h_;    // cross-sectional height aligned with z-axis (bhGiven == 1)
		DVect3 Ydir_; // vector whose projection onto contact plane defines beam y-axis
		double L_;    // beam length
		double kn_;   // normal stiffness [stress/disp]
		double ks_;   // shear  stiffness [stress/disp]
		DVect3 F_;    // beam force (local coord. system, nst, first component positive denotes compression)
		DVect3 M_;    // beam moment (local coord. system, nst)
		double Estr_; // beam strain energy

		// Private state variables:
		double cosAl_;  // cosine of alpha
		double sinAl_;  // sine of alpha
						//   The beam y-axis is oriented at an angle alpha w.r.t. the s-direc.
		bool   cycled_; // True denotes at least one cycle has occurred (with one entry into FDLaw)
		bool   Lset_;   // True denotes that L_ has been set.
		double      userArea_;    // User specified area
		double      rgap_;      // reference gap for the beam part
		void updateMb(const double &Ddss, const double &Ddst);
		void defineTBeamCoordSys(const IContactMechanical *con);
		// Define the beam xyz coordinate system by projecting Ydir_ onto the contact plane to obtain yb (bm_Yaxis),
		// and from yb obtain cosAl_ and sinAl_.
		DVect3 getYdir(const DVect3 &nc) const;
		// Return a unit vector that is not parallel with nc.  
		// First try Ydir_, then try global y and x directions.
		void setL(const IContactMechanical *con);
		void updateEstr();

	};
} // namespace cmodelsxd
// EoF
