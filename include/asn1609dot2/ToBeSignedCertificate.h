/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IEEE1609dot2"
 * 	found in "../IEEE-1609.2-2016.asn1"
 * 	`asn1c -S ../../skeletons -pdu=Certificate -pdu=auto -pdu=Ieee1609Dot2Data -fcompound-names`
 */

#ifndef	_ToBeSignedCertificate_H_
#define	_ToBeSignedCertificate_H_


#include <asn_application.h>

/* Including external dependencies */
#include "CertificateId.h"
#include "HashedId3.h"
#include "CrlSeries.h"
#include "ValidityPeriod.h"
#include "SubjectAssurance.h"
#include <NULL.h>
#include "VerificationKeyIndicator.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct GeographicRegion;
struct SequenceOfPsidSsp;
struct SequenceOfPsidGroupPermissions;
struct PublicEncryptionKey;

/* ToBeSignedCertificate */
typedef struct ToBeSignedCertificate {
	CertificateId_t	 id;
	HashedId3_t	 cracaId;
	CrlSeries_t	 crlSeries;
	ValidityPeriod_t	 validityPeriod;
	struct GeographicRegion	*region	/* OPTIONAL */;
	SubjectAssurance_t	*assuranceLevel	/* OPTIONAL */;
	struct SequenceOfPsidSsp	*appPermissions	/* OPTIONAL */;
	struct SequenceOfPsidGroupPermissions	*certIssuePermissions	/* OPTIONAL */;
	struct SequenceOfPsidGroupPermissions	*certRequestPermissions	/* OPTIONAL */;
	NULL_t	*canRequestRollover	/* OPTIONAL */;
	struct PublicEncryptionKey	*encryptionKey	/* OPTIONAL */;
	VerificationKeyIndicator_t	 verifyKeyIndicator;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ToBeSignedCertificate_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ToBeSignedCertificate;
extern asn_SEQUENCE_specifics_t asn_SPC_ToBeSignedCertificate_specs_1;
extern asn_TYPE_member_t asn_MBR_ToBeSignedCertificate_1[12];
extern asn_per_constraints_t asn_PER_type_ToBeSignedCertificate_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "GeographicRegion.h"
#include "SequenceOfPsidSsp.h"
#include "SequenceOfPsidGroupPermissions.h"
#include "PublicEncryptionKey.h"

#endif	/* _ToBeSignedCertificate_H_ */
#include <asn_internal.h>
