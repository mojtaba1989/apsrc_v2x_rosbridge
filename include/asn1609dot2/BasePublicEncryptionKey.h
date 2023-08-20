/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IEEE1609dot2BaseTypes"
 * 	found in "../IEEE-1609.2-2016.asn1"
 * 	`asn1c -S ../../skeletons -pdu=Certificate -pdu=auto -pdu=Ieee1609Dot2Data -fcompound-names`
 */

#ifndef	_BasePublicEncryptionKey_H_
#define	_BasePublicEncryptionKey_H_


#include <asn_application.h>

/* Including external dependencies */
#include "EccP256CurvePoint.h"
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum BasePublicEncryptionKey_PR {
	BasePublicEncryptionKey_PR_NOTHING,	/* No components present */
	BasePublicEncryptionKey_PR_eciesNistP256,
	BasePublicEncryptionKey_PR_eciesBrainpoolP256r1
	/* Extensions may appear below */
	
} BasePublicEncryptionKey_PR;

/* BasePublicEncryptionKey */
typedef struct BasePublicEncryptionKey {
	BasePublicEncryptionKey_PR present;
	union BasePublicEncryptionKey_u {
		EccP256CurvePoint_t	 eciesNistP256;
		EccP256CurvePoint_t	 eciesBrainpoolP256r1;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} BasePublicEncryptionKey_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_BasePublicEncryptionKey;
extern asn_CHOICE_specifics_t asn_SPC_BasePublicEncryptionKey_specs_1;
extern asn_TYPE_member_t asn_MBR_BasePublicEncryptionKey_1[2];
extern asn_per_constraints_t asn_PER_type_BasePublicEncryptionKey_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _BasePublicEncryptionKey_H_ */
#include <asn_internal.h>
