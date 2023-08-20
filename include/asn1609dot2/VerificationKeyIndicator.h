/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IEEE1609dot2"
 * 	found in "../IEEE-1609.2-2016.asn1"
 * 	`asn1c -S ../../skeletons -pdu=Certificate -pdu=auto -pdu=Ieee1609Dot2Data -fcompound-names`
 */

#ifndef	_VerificationKeyIndicator_H_
#define	_VerificationKeyIndicator_H_


#include <asn_application.h>

/* Including external dependencies */
#include "PublicVerificationKey.h"
#include "EccP256CurvePoint.h"
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum VerificationKeyIndicator_PR {
	VerificationKeyIndicator_PR_NOTHING,	/* No components present */
	VerificationKeyIndicator_PR_verificationKey,
	VerificationKeyIndicator_PR_reconstructionValue
	/* Extensions may appear below */
	
} VerificationKeyIndicator_PR;

/* VerificationKeyIndicator */
typedef struct VerificationKeyIndicator {
	VerificationKeyIndicator_PR present;
	union VerificationKeyIndicator_u {
		PublicVerificationKey_t	 verificationKey;
		EccP256CurvePoint_t	 reconstructionValue;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} VerificationKeyIndicator_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_VerificationKeyIndicator;
extern asn_CHOICE_specifics_t asn_SPC_VerificationKeyIndicator_specs_1;
extern asn_TYPE_member_t asn_MBR_VerificationKeyIndicator_1[2];
extern asn_per_constraints_t asn_PER_type_VerificationKeyIndicator_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _VerificationKeyIndicator_H_ */
#include <asn_internal.h>
